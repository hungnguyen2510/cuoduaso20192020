
import cv2
import numpy as np

IMG_WIDTH = 320
IMG_HEIGHT = 240

class Preprocess:
    def __init__(self):
        self.src_bird_view_point = np.float32([[100,  90],[220,  90],
                                                [0, 170],[320, 170]])

        self.dst_bird_view_point = np.float32([[110,   0], [210,   0],
                                               [100, 240], [220, 240]])

        # self.src_bird_view_point = np.float32([[100,  80],[220,  80],
        #                                         [-30, 195],[350, 195]])

        # self.dst_bird_view_point = np.float32([[100,   0], [220,   0],
        #                                        [100, 240], [220, 240]])

        self.canny_threshold = [150, 300]
        self.gauss_blur_kernel = (3,3)
        self.dilate_kernel = (2,2)

        self.after_birdview_threshold = [10, 255]
        self.after_birdview_erode_kernel = (1, 1)
        self.after_birdview_dilate_kernel = (7, 2)

        self.min_sliding_windows = 10
        self.debug = 1

    def get_BirdView_Matrix(self):
        return cv2.getPerspectiveTransform(self.src_bird_view_point, self.dst_bird_view_point)

    def get_Inv_BirdView_Matrix(self):
        return cv2.getPerspectiveTransform(self.dst_bird_view_point, self.src_bird_view_point)
    
    def create_BirdView_Image(self, img):
        return cv2.warpPerspective(img, self.get_BirdView_Matrix(), (320, 240))

    def find_edge(self, img):
        blur = cv2.GaussianBlur(img, self.gauss_blur_kernel, 0)
        canny = cv2.Canny(blur, self.canny_threshold[0], self.canny_threshold[1], None, 3, False)
        dilate = cv2.dilate(canny, cv2.getStructuringElement(cv2.MORPH_RECT, self.dilate_kernel), iterations=1)
        return blur, canny, dilate

    def afterBirdView(self, img):
        threshold = cv2.threshold(img, self.after_birdview_threshold[0], self.after_birdview_threshold[1], cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
        erosion = cv2.erode(threshold, cv2.getStructuringElement(cv2.MORPH_RECT, self.after_birdview_erode_kernel), iterations=1)
        dilation = cv2.dilate(erosion,cv2.getStructuringElement(cv2.MORPH_RECT, self.after_birdview_dilate_kernel), iterations = 1)
        
        return threshold, erosion, dilation

    def showViewPort(self, img):
        drawed = img.copy()

        p1 = (int(self.src_bird_view_point[0][0]),int(self.src_bird_view_point[0][1]))
        p2 = (int(self.src_bird_view_point[1][0]),int(self.src_bird_view_point[1][1]))
        p3 = (int(self.src_bird_view_point[2][0]),int(self.src_bird_view_point[2][1]))
        p4 = (int(self.src_bird_view_point[3][0]),int(self.src_bird_view_point[3][1]))

        cv2.line(drawed, p1, p2, (0,255,255), 2)
        cv2.line(drawed, p2, p4, (0,255,255), 2)
        cv2.line(drawed, p4, p3, (0,255,255), 2)
        cv2.line(drawed, p3, p1, (0,255,255), 2)

        cv2.circle(drawed,p1,2, (0,0,255), 2 )
        cv2.circle(drawed,p2,2, (0,0,255), 2 )
        cv2.circle(drawed,p3,2, (0,0,255), 2 )
        cv2.circle(drawed,p4,2, (0,0,255), 2 )
        cv2.imshow("showViewPort", drawed)


    def go(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        _, _, dilate = self.find_edge(gray)
        birdview = self.create_BirdView_Image(dilate)
        # birdview_rgb = self.create_BirdView_Image(img)
        _,_,res = self.afterBirdView(birdview)

        if self.debug:
            self.showViewPort(img)
        return res
