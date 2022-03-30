#!/usr/bin/env python
import numpy as np
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
import timm

from timm.data.constants import IMAGENET_DEFAULT_MEAN, IMAGENET_DEFAULT_STD

from external_loop_closure_detection.nearest_neighbors import NearestNeighbors

from cslam_loop_detection.srv import DetectLoopClosure

class L2Norm(nn.Module):
    def __init__(self, dim=1):
        super().__init__()
        self.dim = dim

    def forward(self, input):
        return F.normalize(input, p=2, dim=self.dim)

class ViTLoopClosureDetection(object):
    def __init__(self, params, node):
        self.params = params
        self.node = node
        self.nns = NearestNeighbors()

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")

        encoder_dim = 768
        encoder = timm.create_model('vit_base_patch16_224', pretrained=True)
        
        self.model = nn.Module() 
        self.model.add_module('encoder', encoder)  
        self.model.add_module('pool', nn.Sequential(*[L2Norm()]))

        self.isParallel = False
        self.node.get_logger().info('=> Number of CUDA devices = ' + str(torch.cuda.device_count()))
        if torch.cuda.device_count() > 1:
            self.model.encoder = nn.DataParallel(self.model.encoder)
            self.model.pool = nn.DataParallel(self.model.pool)
            self.isParallel = True     

        if self.params['resume']:
            resume_ckpt = self.params['checkpoint']
            if isfile(resume_ckpt):
                self.node.get_logger().info("=> loading checkpoint '{}'".format(resume_ckpt))
                checkpoint = torch.load(resume_ckpt, map_location=lambda storage, loc: storage)
                start_epoch = checkpoint['epoch']
                best_metric = checkpoint['best_score']
                self.model.load_state_dict(checkpoint['state_dict'])
                self.node.get_logger().info("=> loaded checkpoint '{}' (epoch {})"
                    .format(resume_ckpt, checkpoint['epoch']))   
            else: 
                self.node.get_logger().info("Error: Checkpoint path is incorrect")

        self.model = self.model.to(self.device)
        self.model.eval()
        with torch.no_grad():
            self.node.get_logger().info('====> Extracting Features')
            self.transform = transforms.Compose([
                                transforms.CenterCrop(self.params["crop_size"]),
                                transforms.Resize(224, interpolation=3),
                                transforms.ToTensor(),
                                transforms.Normalize(IMAGENET_DEFAULT_MEAN, IMAGENET_DEFAULT_STD),
                            ])

        self.counter = 0
        os.system('rm best_matches_distances.csv')
        os.system('rm tuples.txt')

    def compute_embedding(self, keyframe):
        with torch.no_grad():    
            image = Image.fromarray(keyframe)
            input = self.transform(image)
            input = torch.unsqueeze(input, 0)
            input = input.to(self.device)
            image_encoding = self.model.encoder.forward_features(input)
            normalized_embedding = self.model.pool(image_encoding) 

            # Compute embedding
            output = normalized_embedding.detach().cpu().numpy()[0]
            
            del input, image_encoding, normalized_embedding, image 
             
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
            if abs(kf - id) < self.params['min_inbetween_keyframes']:
                continue

            self.node.get_logger().info("Match: id0= " + str(id) + ", id1= " + str(kf) + ", distance= " + str(d))
            f = open("best_matches_distances.csv", "a")
            f.write(str(id)+","+str(kf) +","+str(d)+'\n')
            f.close()

            if d > self.params['threshold']:
                continue
    
            return kf, kfs
        return None, None

    def detect_loop_closure_service(self, req, res):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(req.image.image, desired_encoding='passthrough')
        embedding = self.compute_embedding(cv_image)

        # Netvlad processing
        match = None
        if self.counter > 0:
            match, best_matches = self.detect(embedding, req.image.id) # Systematic evaluation
        self.add_keyframe(embedding, req.image.id)
        self.counter = self.counter + 1

        if match is not None:
            res.is_detected=True
            res.detected_loop_closure_id=match
            res.best_matches=best_matches
        else:
            res.is_detected=False
            res.detected_loop_closure_id=-1
            res.best_matches=[]

        return res