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
from cslam.vpr.cosplace_utils.network import GeoLocalizationNet
from ament_index_python.packages import get_package_share_directory

IMAGENET_DEFAULT_MEAN = (0.485, 0.456, 0.406)
IMAGENET_DEFAULT_STD = (0.229, 0.224, 0.225)


class CosPlace(object):
    """CosPlace matcher
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

            if torch.cuda.is_available():
                self.device = torch.device("cuda")
            else:
                self.device = torch.device("cpu")

            self.descriptor_dim = self.params[
                'frontend.cosplace.descriptor_dim']
            self.model = GeoLocalizationNet(
                self.params['frontend.cosplace.backbone'], self.descriptor_dim,
                node)

            resume_ckpt = self.params['frontend.nn_checkpoint']
            if isfile(resume_ckpt):
                self.node.get_logger().info("loading checkpoint '{}'".format(resume_ckpt))
                checkpoint = torch.load(
                    resume_ckpt, map_location=lambda storage, loc: storage)

                self.model.load_state_dict(checkpoint)
                self.model = self.model.to(self.device)
            else:
                self.node.get_logger().error("Error: Checkpoint path is incorrect {}".format(resume_ckpt))
                exit()

            self.model.eval()
            self.transform = transforms.Compose([
                transforms.CenterCrop(self.params["frontend.image_crop_size"]),
                transforms.Resize(224, interpolation=3),
                transforms.ToTensor(),
                transforms.Normalize(IMAGENET_DEFAULT_MEAN,
                                     IMAGENET_DEFAULT_STD),
            ])

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

                image_encoding = self.model.forward(input)

                output = image_encoding[0].detach().cpu().numpy()
                del input, image_encoding, image
            return output
        else:
            # Random descriptor if disabled
            # Use this option only for testing
            return np.random.rand(self.descriptor_dim)
