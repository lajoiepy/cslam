import numpy as np

import os
from os.path import join, exists, isfile, realpath, dirname
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.autograd import Variable
from torch.utils.data import DataLoader, SubsetRandomSampler
from torch.utils.data.dataset import Subset
import torchvision.transforms as transforms
from PIL import Image
from datetime import datetime
import torchvision.datasets as datasets
import torchvision.models as models
import numpy as np
import sys
import pickle
import sklearn
from sklearn.neighbors import NearestNeighbors
from ament_index_python.packages import get_package_share_directory

IMAGENET_DEFAULT_MEAN = (0.485, 0.456, 0.406)
IMAGENET_DEFAULT_STD = (0.229, 0.224, 0.225)


class NetVLADLayer(nn.Module):
    """ NetVLAD layer implementation
        based on https://github.com/lyakaap/NetVLAD-pytorch/blob/master/netvlad.py
    """

    def __init__(self,
                 num_clusters=64,
                 dim=128,
                 normalize_input=True,
                 vladv2=False):
        """
        Args:
            num_clusters : int
                The number of clusters
            dim : int
                Dimension of descriptors
            alpha : float
                Parameter of initialization. Larger value is harder assignment.
            normalize_input : bool
                If true, descriptor-wise L2 normalization is applied to input.
            vladv2 : bool
                If true, use vladv2 otherwise use vladv1
        """
        super(NetVLADLayer, self).__init__()
        self.num_clusters = num_clusters
        self.dim = dim
        self.alpha = 0
        self.vladv2 = vladv2
        self.normalize_input = normalize_input
        self.conv = nn.Conv2d(dim,
                              num_clusters,
                              kernel_size=(1, 1),
                              bias=vladv2)
        self.centroids = nn.Parameter(torch.rand(num_clusters, dim))

    def init_params(self, clsts, traindescs):
        if self.vladv2 == False:
            clstsAssign = clsts / np.linalg.norm(clsts, axis=1, keepdims=True)
            dots = np.dot(clstsAssign, traindescs.T)
            dots.sort(0)
            dots = dots[::-1, :]  # sort, descending

            self.alpha = (-np.log(0.01) /
                          np.mean(dots[0, :] - dots[1, :])).item()
            self.centroids = nn.Parameter(torch.from_numpy(clsts))
            self.conv.weight = nn.Parameter(
                torch.from_numpy(self.alpha *
                                 clstsAssign).unsqueeze(2).unsqueeze(3))
            self.conv.bias = None
        else:
            knn = NearestNeighbors(n_jobs=-1)
            knn.fit(traindescs)
            del traindescs
            dsSq = np.square(knn.kneighbors(clsts, 2)[1])
            del knn
            self.alpha = (-np.log(0.01) /
                          np.mean(dsSq[:, 1] - dsSq[:, 0])).item()
            self.centroids = nn.Parameter(torch.from_numpy(clsts))
            del clsts, dsSq

            self.conv.weight = nn.Parameter(
                (2.0 * self.alpha *
                 self.centroids).unsqueeze(-1).unsqueeze(-1))
            self.conv.bias = nn.Parameter(-self.alpha *
                                          self.centroids.norm(dim=1))

    def forward(self, x):
        """Forward pass through the NetVLAD network

        Args:
            x (image): image to match

        Returns:
            torch array: Global image descriptor
        """
        N, C = x.shape[:2]

        if self.normalize_input:
            x = F.normalize(x, p=2, dim=1)  # across descriptor dim

        # soft-assignment
        soft_assign = self.conv(x).view(N, self.num_clusters, -1)
        soft_assign = F.softmax(soft_assign, dim=1)

        x_flatten = x.view(N, C, -1)

        # calculate residuals to each clusters
        vlad = torch.zeros([N, self.num_clusters, C],
                           dtype=x.dtype,
                           layout=x.layout,
                           device=x.device)
        for C in range(self.num_clusters
                       ):  # slower than non-looped, but lower memory usage
            residual = x_flatten.unsqueeze(0).permute(1, 0, 2, 3) - \
                    self.centroids[C:C+1, :].expand(x_flatten.size(-1), -1, -1).permute(1, 2, 0).unsqueeze(0)
            residual *= soft_assign[:, C:C + 1, :].unsqueeze(2)
            vlad[:, C:C + 1, :] = residual.sum(dim=-1)

        vlad = F.normalize(vlad, p=2, dim=2)  # intra-normalization
        vlad = vlad.view(x.size(0), -1)  # flatten
        vlad = F.normalize(vlad, p=2, dim=1)  # L2 normalize

        return vlad


class NetVLAD(object):
    """NetVLAD matcher
    """

    def __init__(self, params, node):
        """Initialization

        Args:
            params (dict): parameters
        """
        self.params = params
        self.node = node

        self.enable = self.params['frontend.nn_checkpoint'].lower(
        ) != 'disable'
        if self.enable:
            pkg_folder = get_package_share_directory("cslam")
            self.params['frontend.nn_checkpoint'] = join(
                pkg_folder, self.params['frontend.nn_checkpoint'])
            self.params['frontend.netvlad.pca_checkpoint'] = join(
                pkg_folder,
                self.node.get_parameter(
                    'frontend.netvlad.pca_checkpoint').value)

            if torch.cuda.is_available():
                self.device = torch.device("cuda")
            else:
                self.device = torch.device("cpu")

            encoder_dim = 512
            encoder = models.vgg16(pretrained=True)
            # capture only feature part and remove last relu and maxpool
            layers = list(encoder.features.children())[:-2]
            # if using pretrained then only train conv5_1, conv5_2, and conv5_3
            for l in layers[:-5]:
                for p in l.parameters():
                    p.requires_grad = False

            encoder = nn.Sequential(*layers)
            self.model = nn.Module()
            self.model.add_module('encoder', encoder)
            netvlad_layer = NetVLADLayer(num_clusters=64,
                                         dim=encoder_dim,
                                         vladv2=False)
            self.model.add_module('pool', netvlad_layer)

            self.isParallel = False
            print('=> Number of CUDA devices = ' +
                  str(torch.cuda.device_count()))
            if torch.cuda.device_count() > 1:
                self.model.encoder = nn.DataParallel(self.model.encoder)
                self.model.pool = nn.DataParallel(self.model.pool)
                self.isParallel = True

            resume_ckpt = self.params['frontend.nn_checkpoint']
            if isfile(resume_ckpt):
                print("=> loading checkpoint '{}'".format(resume_ckpt))
                checkpoint = torch.load(
                    resume_ckpt, map_location=lambda storage, loc: storage)
                start_epoch = checkpoint['epoch']
                best_metric = checkpoint['best_score']
                self.model.load_state_dict(checkpoint['state_dict'])
                self.model = self.model.to(self.device)
                print("=> loaded checkpoint '{}' (epoch {})".format(
                    resume_ckpt, checkpoint['epoch']))
            else:
                print("Error: Checkpoint path is incorrect")

            self.model.eval()
            self.transform = transforms.Compose([
                transforms.CenterCrop(self.params["frontend.image_crop_size"]),
                transforms.Resize(224, interpolation=3),
                transforms.ToTensor(),
                transforms.Normalize(IMAGENET_DEFAULT_MEAN,
                                     IMAGENET_DEFAULT_STD),
            ])
            self.pca = pickle.load(
                open(self.params['frontend.netvlad.pca_checkpoint'], 'rb'))

    def compute_embedding(self, keyframe):
        """Load image to device and extract the global image descriptor

        Args:
            keyframe (image): image to match

        Returns:
            np.array: global image descriptor
        """
        if self.enable:
            with torch.no_grad():
                image = Image.fromarray(keyframe)
                input = self.transform(image)
                input = torch.unsqueeze(input, 0)
                input = input.to(self.device)
                image_encoding = self.model.encoder(input)
                vlad_encoding = self.model.pool(image_encoding)

                # Compute NetVLAD
                embedding = vlad_encoding.detach().cpu().numpy()

                # Run PCA transform
                reduced_embedding = self.pca.transform(embedding)
                normalized_embedding = sklearn.preprocessing.normalize(
                    reduced_embedding)
                output = normalized_embedding[0]

                del input, image_encoding, vlad_encoding, reduced_embedding, normalized_embedding, image

            return output
        else:
            # Random descriptor if disabled
            # Use this option only for testing
            return np.random.rand(128)
