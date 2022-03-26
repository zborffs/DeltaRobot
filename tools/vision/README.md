Computer Vision
===============
This directory contains the C++ code responsible for:
1. Working out the chess position recognition (CPR) pipeline. Going from an image to a FEN string.
2. Retraining RESNET on chess piece images.
3. Assessing the overall robustness and run-time performance of the pipeline.

Dependencies
------------
1. OpenCV (Classical CV Techniques; image representation)
2. TensorFlow (CNN)

Pipeline
--------
1. Identify corners of chess board (corner detection?)
2. Crop out everything but board
3. Hough transform
4. Find intersection of hough lines. those are corners of squares
5. crop each square to get 64 subimages
6. pass subimages into classifier to figure out what piece is on each square
7. return fen
