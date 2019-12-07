class LaneDetector:
    def __init__(self, camera, ROItop=100, ROIleft=0, ROIwidth=480, ROIheight=100):
        self.camera = camera
        self.ROItop = ROItop
        self.ROIleft = ROIleft
        self.ROIwidth = ROIwidth
        self.ROIheight = ROIheight
        self.markedImage = self.camera.value ## type:numpy ndarray
        self.ROIimage = None
        self.count = 1000
        self.markROI()

    def markROI(self):
        width = 2
        top = self.ROItop
        bottom = self.ROItop + self.ROIheight
        left = self.ROIleft
        right = self.ROIleft + self.ROIwidth

        # mark left border
        self.markedImage[top:bottom][left:left+width][0] = 255
        self.markedImage[top:bottom][left:left+width][1] = 0
        self.markedImage[top:bottom][left:left+width][2] = 0
        # mark right border
        self.markedImage[top:bottom][right-width:right][0] = 255
        self.markedImage[top:bottom][right-width:right][1] = 0
        self.markedImage[top:bottom][right-width:right][2] = 0
        # mark top border
        self.markedImage[top:top+width][left:right][0] = 255
        self.markedImage[top:top+width][left:right][1] = 0
        self.markedImage[top:top+width][left:right][2] = 0
        # mark bottom border
        self.markedImage[bottom:bottom-width][left:right][0] = 255
        self.markedImage[bottom:bottom-width][left:right][1] = 0
        self.markedImage[bottom:bottom-width][left:right][2] = 0

    def ROIToFile(self):
        self.count += 1
        cv2.imwrite('./img/img' + str(self.count) + '.jpg', self.markedImage)
