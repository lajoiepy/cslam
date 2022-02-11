#!/usr/bin/env python
import numpy as np
import rospy
from cv_bridge import CvBridge

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
sys.path.append('/home/lajoiepy/Documents/projects/SelfSupervisedPlaceRecognition/pytorch-NetVlad') # TODO: add as submodule?
import netvlad
import pickle
import sklearn

from timm.data.constants import IMAGENET_DEFAULT_MEAN, IMAGENET_DEFAULT_STD

from external_loop_closure_detection.nearest_neighbors import NearestNeighbors

from external_loop_closure_detection.srv import DetectLoopClosure, DetectLoopClosureResponse


class NetVLADLoopClosureDetection(object):
    def __init__(self, params):
        self.params = params
        self.nns = NearestNeighbors()

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
        net_vlad = netvlad.NetVLAD(num_clusters=64, dim=encoder_dim, vladv2=False)
        self.model.add_module('pool', net_vlad)

        self.isParallel = False
        print('=> Number of CUDA devices = ' + str(torch.cuda.device_count()))
        if torch.cuda.device_count() > 1:
            self.model.encoder = nn.DataParallel(self.model.encoder)
            self.model.pool = nn.DataParallel(self.model.pool)
            self.isParallel = True     

        resume_ckpt = self.params['checkpoint']
        if isfile(resume_ckpt):
            print("=> loading checkpoint '{}'".format(resume_ckpt))
            checkpoint = torch.load(resume_ckpt, map_location=lambda storage, loc: storage)
            start_epoch = checkpoint['epoch']
            best_metric = checkpoint['best_score']
            self.model.load_state_dict(checkpoint['state_dict'])
            self.model = self.model.to(self.device)
            print("=> loaded checkpoint '{}' (epoch {})"
                  .format(resume_ckpt, checkpoint['epoch']))   
        else: 
            print("Error: Checkpoint path is incorrect")

        self.model.eval()
        with torch.no_grad():
            print('====> Extracting Features')
            pool_size = encoder_dim
            pool_size *= 64

            self.transform = transforms.Compose([
                                transforms.CenterCrop(self.params["crop_size"]),
                                transforms.Resize(224, interpolation=3),
                                transforms.ToTensor(),
                                transforms.Normalize(IMAGENET_DEFAULT_MEAN, IMAGENET_DEFAULT_STD),
                            ])
        
        self.pca = pickle.load(open(self.params["pca"],'rb'))
        self.counter = 0
        os.system('rm best_matches_netvlad_distances.csv')
        os.system('rm tuples.txt')

    def compute_embedding(self, keyframe):
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
            normalized_embedding = sklearn.preprocessing.normalize(reduced_embedding)
            output = normalized_embedding[0]
            
            del input, image_encoding, vlad_encoding, reduced_embedding, normalized_embedding, image 
            
        return output

    def add_keyframe(self, embedding, id):
        self.nns.add_item(embedding, id)

    def detect(self, embedding, id):
        kfs, ds = self.nns.search(embedding, k=self.params['nb_best_matches'])

        if len(kfs) > 0 and kfs[0] == id:
            kfs, ds = kfs[1:], ds[1:]
        if len(kfs) == 0:
            return None

        for kf, d in zip(kfs, ds):
            #TODO: restablish condition
            if abs(kf - id) < self.params['min_inbetween_keyframes']:
                continue

            rospy.loginfo("Match: id0= " + str(kf) + ", id1= " + str(id) + ", distance= " + str(d))
            f = open("best_matches_distances.csv", "a")
            f.write(str(id)+","+str(kf) +","+str(d)+'\n')
            f.close()

            if d > self.params['threshold']:
                continue
    
            return kf, kfs
        return None, None

    def detect_loop_closure_service(self, req):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(req.image, desired_encoding='passthrough')
        embedding = self.compute_embedding(cv_image)

        # Netvlad processing
        match = None
        if self.counter > 0:
            match, best_matches = self.detect(embedding, req.image.header.seq) # Systematic evaluation
        self.add_keyframe(embedding, req.image.header.seq)
        self.counter = self.counter + 1

        if match is not None:
            return DetectLoopClosureResponse(is_detected=True, detected_loop_closure_id=match, best_matches=np.asarray(best_matches))
        else:
            return DetectLoopClosureResponse(is_detected=False, detected_loop_closure_id=-1, best_matches=np.array([]))