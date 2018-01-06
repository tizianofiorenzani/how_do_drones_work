-----------------------
Calibration procedure
-----------------------

1) Print the chessBoard.jpg without any adaption to the page
2) Accurately measure the side of the printed chess board
3) Mount the chess board on a rigid and flat panel
4) Take multiple pictures of the chess board with the camera. In order to get good results, take at least 20 different pictures varying the angles and distance. You can use the python script save_snapshots.py
5) put all the images in a folder

6) Open a terminal
cd to this folder
execute the cameracalib.py by calling: cameracalib.py  <folder> <image type> <num rows> <num cols> <cell dimension>
 - folder: relative path to the folder containing the pictures
 - image type: the image type, like png or jpg or bmp
 - num rows: the number of rows of the checker board
 - num cols: the number of columns of the checker board
 - cell dimension: the physical dimension of a cell side in mm

7) Use the output files
