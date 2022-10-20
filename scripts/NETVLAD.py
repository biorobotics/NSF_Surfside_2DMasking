from pathlib import Path
from pprint import pformat
import os
import shutil
from hloc import extract_features, match_features, pairs_from_covisibility, pairs_from_retrieval
from hloc import colmap_from_nvm, triangulation, localize_sfm, visualization
import ipdb

# image path
dataset = Path('/home/biorobotics/Documents/col_ws/')  # change this if your dataset is somewhere else
images = dataset / 'pairs/'

# output path
outputs = Path('/home/biorobotics/Documents/col_ws/')  # where everything will be saved
sfm_pairs = outputs / 'pairs-db-covis20.txt'  # top 20 most covisible in SIFT model


for i in range(222, 242):
    t = i + 1
    original = "/home/biorobotics/Documents/col_ws/sortie28/S3_3DGrid(" + str(t) + ").JPG"
    target = str(images) + "/S4_3DGrid (" + str(t) + ").JPG"
    retrieval_conf = extract_features.confs['netvlad']
    feature_conf = extract_features.confs['superpoint_aachen']
    matcher_conf = match_features.confs['superglue']
    shutil.copy(original, target)
    retrieval_path = extract_features.main(retrieval_conf, images, outputs, overwrite=True)
    pairs_from_retrieval.main(retrieval_path, sfm_pairs, num_matched=5)
    os.remove(retrieval_path)
    os.remove(str(images) + "/S3_3DGrid (" + str(i + 1) + ").JPG")
