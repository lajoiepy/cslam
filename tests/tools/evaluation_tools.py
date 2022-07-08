#
# Script to convert .g2o files into .tum format
# Author: Pierre-Yves Lajoie
#
import glob
import os
import sys
import subprocess


def convert_g2o_to_tum(input_filename, output_filename):
    with open(os.path.join(folder, input_filename), 'r') as fi:
        content = fi.readlines()
        with open(output_filename, 'w') as fo:
            for line in content:
                if line[0] == 'V':
                    elems = line.split()
                    tum_line = ""
                    for elem in elems[1:]:
                        tum_line += elem + " "
                    tum_line_to_write = tum_line[0:-1] + '\n'
                    fo.write(tum_line_to_write)


def convert_tum_to_g2o(input_filename, output_filename):
    with open(os.path.join(os.getcwd(), input_filename), 'r') as fi:
        content = fi.readlines()
        with open(output_filename, 'w') as fo:
            for line in content:
                fo.write("VERTEX_SE3:QUAT " + line)


def evaluate_ape(input_filename, gt_filename):
    return subprocess.check_output("evo_ape tum " + gt_filename + " " +
                                   input_filename + " -a",
                                   shell=True)
