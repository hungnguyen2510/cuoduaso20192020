import numpy as np 
import cv2

class Window:
    def __init__(self, H_pos = None, W_pos = None, moment = None):
        self.H_pos = H_pos
        self.W_pos = W_pos
        self.moment = moment

class DynamicSlidingWindow:
    def __init__(self, root, binaryImg):
        self.max_H_window = 10 #
        self.min_W_window = 50
        self.windows = [Window(
                H_pos = [root[1] - 1, root[1] - 1],
                W_pos = [root[0] - self.min_W_window // 2 , root[0] + self.min_W_window // 2],
                moment= root)]
        self.binaryImg = binaryImg.copy()
        self.binaryImgShape = binaryImg.shape[:2]

    def next_window_position(self):
        last_window = self.windows[-1]
        H_start = last_window.H_pos[0] - self.max_H_window
        H_end = last_window.H_pos[0]
        if len(self.windows) > 2:
            last_window_2 = self.windows[len(self.windows) - 2]
            dx = last_window.moment[0] - last_window_2.moment[0]
            move_momentX = last_window.moment[0] + dx
            W_start = move_momentX - self.min_W_window//2 - abs(dx)
            W_end = move_momentX + self.min_W_window//2 + abs(dx)
        else:
            W_start = last_window.moment[0] - self.min_W_window//2
            W_end = last_window.moment[0] + self.min_W_window//2
        
        H_start = 0 if H_start < 0 else H_start
        H_end = 0 if H_end < 0 else H_end
        W_end = 0 if W_end < 0 else W_end
        W_start = 0 if W_start < 0 else W_start

        H_start = self.binaryImgShape[0] if H_start > self.binaryImgShape[0] else H_start
        H_end = self.binaryImgShape[0] if H_end > self.binaryImgShape[0] else H_end
        W_end = self.binaryImgShape[1] if W_end > self.binaryImgShape[1] else W_end
        W_start = self.binaryImgShape[1] if W_start > self.binaryImgShape[1] else W_start

        return H_start, H_end, W_start, W_end

    def slide(self):
        count_disappear = 0
        while True:
            H_start, H_end, W_start, W_end = self.next_window_position()
            mask = self.binaryImg[H_start : H_end, W_start : W_end]
            M = cv2.moments(mask)
            
            if M['m00'] == 0:
                break
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            self.windows.append(Window(
                H_pos = [H_start, H_end],
                W_pos = [W_start, W_end],
                moment= [W_start + cx, H_start + cy]))
            
            if H_start <= 0:
                break
            
        return self.windows
        