import cv2, time, math
import numpy as np 
from lib.preprocess import Preprocess
import lib.util as util
from lib.sliding import DynamicSlidingWindow

def func(x,a,b):
    return a*np.exp(b/x)

class LaneDetect:
    def __init__(self):
        self.preProcess = Preprocess()
        
        self.debug = 1
        # dotted line detect params
        self.dotted_edge_height = [20, 60]
        self.distance_between_2_dotted_segment = 60
        self.max_dotted_segment = 3
        self.move_mid = 40
        self.line_width = 80
        # solid line detect params

    def create_mask_line(self, cnts):
        mask = np.zeros((240,320), dtype=np.uint8)
        for cnt in cnts:
            cv2.drawContours(mask,[cnt],0,255,-1)
        return mask

    def fit_line_from_a_contour(self, cnt):
        line = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
        vx = line[0]
        vy = line[1]
        x = line[2]
        y = line[3]

        lefty = (-x * vy / vx) + y
        righty = ((320 - x) * vy / vx) + y
        point1 = (320 - 1, righty)
        point2 = (0, lefty)
        return lefty, righty, point1, point2

        
    def get_dotted_solid_contours(self, img):
        cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        dotted_cnts = []
        solid_cnts = []
        solid_and_dotted_cnts = []

        for cnt in cnts:
            minrect = cv2.minAreaRect(cnt)
            box = np.int0(cv2.boxPoints(minrect))
            edge1 = cv2.norm(box[1] - box[0])
            edge2 = cv2.norm(box[2] - box[1])
            longest_edge = edge1 if edge1 > edge2 else edge2
            if longest_edge > self.dotted_edge_height[1]:
                solid_cnts.append(cnt)
                solid_and_dotted_cnts.append(cnt)
            if longest_edge >= self.dotted_edge_height[0] and longest_edge <= self.dotted_edge_height[1]:
                dotted_cnts.append(cnt)
                solid_and_dotted_cnts.append(cnt)
        return dotted_cnts, solid_cnts, solid_and_dotted_cnts

    def find_dotted_line(self, birdview_rgb, dotted_cnts):
        if len(dotted_cnts) < self.max_dotted_segment:
            return []
        
        dotted_holder = []
        for cnt in dotted_cnts:
            extTop = cnt[cnt[:, :, 1].argmin()][0] #[x,y]
            extBot = cnt[cnt[:, :, 1].argmax()][0] #[x,y]
            # edge, angle_edge = util.get_angle_longest_edge_from_contour(cnt)

            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            dotted_holder.append([cx, cy, extBot[0], extBot[1], extTop[0], extTop[1]])

        dotted_holder = np.array(dotted_holder)
        # dotted_holder = dotted_holder[dotted_holder[:,3].argsort()[::-1]]

        dotted_segments = []
        for i in range(len(dotted_holder)):
            segment = [dotted_holder[i]]
            tail = dotted_holder[i]
            for j in range(i+1, len(dotted_holder)):
                head = dotted_holder[j]
                distance = cv2.norm(np.array([tail[4], tail[5]]) - np.array([head[2], head[3]]))
                if distance <= self.distance_between_2_dotted_segment:
                    segment.append(dotted_holder[j])
                    tail = dotted_holder[j]
            if len(segment) >= self.max_dotted_segment:
                dotted_segments.append(segment)
        
        if len(dotted_segments) < 1:
            return []
        
        max_len_seg = len(dotted_segments[0])
        max_segment = dotted_segments[0]
        for i in range(1, len(dotted_segments)):
            if len(dotted_segments[i]) > max_len_seg:
                max_segment = dotted_segments[i]
                max_len_seg = len(dotted_segments[i])
        return max_segment
        # x_arr = []
        # y_arr = []
        # for seg in max_segment:
        #     x_arr.append(seg[0])
        #     x_arr.append(seg[2])
        #     x_arr.append(seg[4])

        #     y_arr.append(120 + abs(120 - seg[1]) if seg[1] <= 120 else 120 - abs(120 - seg[1]))
        #     y_arr.append(120 + abs(120 - seg[3]) if seg[3] <= 120 else 120 - abs(120 - seg[3]))
        #     y_arr.append(120 + abs(120 - seg[5]) if seg[5] <= 120 else 120 - abs(120 - seg[5]))
        
        # coeff = np.polyfit(np.array(y_arr), np.array(x_arr), 2)

        # ys = np.arange(1, 240, 1)
        # # xs = coeff[0] * (ys**2) + coeff[1] * ys + 160
        # xs = np.polyval(coeff, ys)
        # xs = xs.astype(np.int32)
        # for i in range(len(xs)):
        #     x = xs[i]
        #     y = ys[i]
        #     y = 120 + abs(120 - y) if y <= 120 else 120 - abs(120 - y)
        #     if self.debug:
        #         cv2.circle(birdview_rgb, (x, y), 2, (0,255,0), 2)


    def find_solid_line(self, solid_mask, solid_cnts):
        left_extBottom, right_extBottom = None, None
        for cnt in solid_cnts:
            extBot = cnt[cnt[:, :, 1].argmax()][0]
            if extBot[0] > 160:
                if right_extBottom is None:
                    right_extBottom = extBot
                else:
                    if extBot[1] > right_extBottom[1]:
                        right_extBottom = extBot
                    
            if extBot[0] < 160:
                if left_extBottom is None:
                    left_extBottom = extBot
                else:
                    if extBot[1] > left_extBottom[1]:
                        left_extBottom = extBot

        
        left_windows = [] if left_extBottom is None else DynamicSlidingWindow(left_extBottom, solid_mask).slide()
        right_windows = [] if right_extBottom is None else DynamicSlidingWindow(right_extBottom, solid_mask).slide()
        if self.debug:
            drawed = cv2.cvtColor(solid_mask, cv2.COLOR_GRAY2RGB)
            # color = (np.random.randint(0,255),np.random.randint(0,255),np.random.randint(0,255))
            color_left = (255,255,0)
            color_right = (255,0,255)
            for wind in left_windows:
                cv2.rectangle(drawed, (wind.W_pos[0], wind.H_pos[0]), (wind.W_pos[1], wind.H_pos[1]) ,color_left , 1)
                
            color = (np.random.randint(0,255),np.random.randint(0,255),np.random.randint(0,255))
            for wind in right_windows:
                cv2.rectangle(drawed, (wind.W_pos[0], wind.H_pos[0]), (wind.W_pos[1], wind.H_pos[1]) ,color_right , 1)

            cv2.imshow("drawed", drawed)
        
        solid_left = [list(window.moment) for window in left_windows]
        solid_right = [list(window.moment) for window in right_windows]
        return solid_left, solid_right

        # for cnt in solid_cnts:
            # hull = cv2.convexHull(cnt,returnPoints = False)
            # defects = cv2.convexityDefects(cnt,hull)
            # for i in range(defects.shape[0]):
            #     s,e,f,d = defects[i,0]
            #     end = tuple(cnt[e][0])
            #     far = tuple(cnt[f][0])
            #     cv2.circle(drawed,far,5,[0,0,255],-1)
            
            # epsilon = 0.01*cv2.arcLength(cnt,True)
            # approx = cv2.approxPolyDP(cnt,epsilon,True)
            # hull = cv2.convexHull(cnt)
            # cv2.drawContours(drawed, approx, 3, (0, 0, 255), 5)
            # extTop = cnt[cnt[:, :, 1].argmin()][0] #[x,y]
            # extBot = cnt[cnt[:, :, 1].argmax()][0] #[x,y]
            # windows = DynamicSlidingWindow(extBot, solid_mask).slide()
            # if self.debug:
            #     drawed = cv2.cvtColor(solid_mask, cv2.COLOR_GRAY2RGB)
            #     color = (np.random.randint(0,255),np.random.randint(0,255),np.random.randint(0,255))
            #     for wind in windows:
            #         cv2.rectangle(drawed, (wind.W_pos[0], wind.H_pos[0]), (wind.W_pos[1], wind.H_pos[1]) ,color , 1)
            #     cv2.imshow("drawed", drawed)

    # def find_solid_line_v2(self, binary, cnts):
    #     drawed = cv2.cvtColor(binary, cv2.COLOR_GRAY2RGB)
    #     num = 20
    #     pix_per = 240//num

    #     arr_moments = []
    #     for i in range(num, 0, -1):
    #         moments = []
    #         mask = binary[(num-1)*pix_per-1: num*pix_per+1, :]
    #         cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
    #         for cnt in cnts:
    #             M = cv2.moment(cnt)
            #     if len(moments) > 0:
            #     else:
            
            # moments.append(M)
            # arr_moments.append(moments)
    

    def calc_line_properties(self, solid_left, solid_right, dotted_line):
        # print(solid_right[-1], solid_right[0])
        line_left_properties, line_mid_properties, line_right_properties = self.cal_solid_angle_curve(solid_left, solid_right)
        return line_left_properties, line_mid_properties, line_right_properties, self.calc_dotted_angle_curve(dotted_line)
    
    def calc_dotted_angle_curve(self, points):
        if len(points) < 3:
            return 999, 999, 999
        point_top, point_mid, point_bot = points[0], np.median(points, axis=0), points[-1]
        angle = point_mid[0] - 160
  
        #calc curve
        point_bot[1] = 120 + abs(120 - point_bot[1]) if point_bot[1] <= 120 else 120 - abs(120 - point_bot[1])
        point_mid[1] = 120 + abs(120 - point_mid[1]) if point_mid[1] <= 120 else 120 - abs(120 - point_mid[1])
        point_top[1] = 120 + abs(120 - point_top[1]) if point_top[1] <= 120 else 120 - abs(120 - point_top[1])

        coeff1 = np.polyfit([point_bot[1], point_mid[1]], [point_bot[0], point_mid[0]], 1)
        coeff2 = np.polyfit([point_mid[1], point_top[1]], [point_mid[0], point_top[0]], 1)

        c1 = math.degrees(math.atan(coeff1[0]))
        c2 = math.degrees(math.atan(coeff2[0]))
        c = c1 + c2

        return angle, c, (240 - point_bot[1])

    def calc_solid_line_angle_curve(self, points, side):
        if len(points) < 3:
            return 999 * side, 999 * side, 999 * side
        point_top, point_mid, point_bot = points[0], np.median(points, axis=0), points[-1]
        goal_point = point_mid[0] - self.move_mid * side
        angle = goal_point - 160

        #calc curve
        point_bot[1] = 120 + abs(120 - point_bot[1]) if point_bot[1] <= 120 else 120 - abs(120 - point_bot[1])
        point_mid[1] = 120 + abs(120 - point_mid[1]) if point_mid[1] <= 120 else 120 - abs(120 - point_mid[1])
        point_top[1] = 120 + abs(120 - point_top[1]) if point_top[1] <= 120 else 120 - abs(120 - point_top[1])

        coeff1 = np.polyfit([point_bot[1], point_mid[1]], [point_bot[0], point_mid[0]], 1)
        coeff2 = np.polyfit([point_mid[1], point_top[1]], [point_mid[0], point_top[0]], 1)

        c1 = math.degrees(math.atan(coeff1[0]))
        c2 = math.degrees(math.atan(coeff2[0]))
        c = c1 + c2

        return angle, c, (240 - point_bot[1])
    
    def cal_solid_angle_curve(self, lpoints, rpoints):
        def cal_curve(point_top_t, point_mid_t, point_bot_t):
            
            point_bot = point_bot_t[:]
            point_top = point_top_t[:]
            point_mid = point_mid_t[:]

            point_bot[1] = 120 + abs(120 - point_bot[1]) if point_bot[1] <= 120 else 120 - abs(120 - point_bot[1])
            point_mid[1] = 120 + abs(120 - point_mid[1]) if point_mid[1] <= 120 else 120 - abs(120 - point_mid[1])
            point_top[1] = 120 + abs(120 - point_top[1]) if point_top[1] <= 120 else 120 - abs(120 - point_top[1])

            coeff1 = np.polyfit([point_bot[1], point_mid[1]], [point_bot[0], point_mid[0]], 1)
            coeff2 = np.polyfit([point_mid[1], point_top[1]], [point_mid[0], point_top[0]], 1)

            c1 = math.degrees(math.atan(coeff1[0]))
            c2 = math.degrees(math.atan(coeff2[0]))
            c = c1 + c2

            return c

        def cal_angle(point_midx, side):
            if side == 0:
                angle = point_midx - 160
            else:
                goal_point = point_midx - self.line_width//4 * side
                angle = goal_point - 160
            return angle

        l_angle, l_curve, l_dis, l_far = -999,-999,-999, -999

        r_angle, r_curve, r_dis,r_far = 999,999,999, 999

        m_angle, m_curve, m_dis,m_far = 999,999,999,999

        l_p_top, l_p_mid, l_p_bot = None, None, None
        r_p_top, r_p_mid, r_p_bot = None, None, None
        
        
        if len(lpoints) > 3: 
            l_p_top, l_p_mid, l_p_bot = lpoints[-1], np.median(lpoints, axis=0), lpoints[0]
            l_angle = cal_angle(l_p_mid[0], -1)
            l_curve = cal_curve(l_p_top, l_p_mid, l_p_bot)
            l_dis = 240 - l_p_bot[1]
            l_far = l_p_bot[0] - 160

        if len(rpoints) > 3:
            r_p_top, r_p_mid, r_p_bot = rpoints[-1], np.median(rpoints, axis=0), rpoints[0]
            r_angle = cal_angle(r_p_mid[0], 1)
            r_curve = cal_curve(r_p_top, r_p_mid, r_p_bot)
            r_dis = 240 - r_p_bot[1]
            r_far = 160 - r_p_bot[0]

        if l_p_mid is not None and r_p_mid is not None:
            m_angle = cal_angle((l_p_mid[0] + r_p_mid[0])//2, 0)
            m_curve = (l_curve + r_curve) // 2
            m_dis = min([l_dis, r_dis])
            self.line_width = abs(l_p_mid[0] - r_p_mid[0])
            m_far = (l_p_bot[0] + r_p_bot[0]) // 2 - 160
        
        if l_p_mid is not None and r_p_mid is None:
            m_angle = cal_angle(l_p_mid[0] + self.line_width//2, 0)
            m_curve = l_curve
            m_dis = l_dis
            m_far = (l_p_bot[0] + self.line_width//2) - 160
        if l_p_mid is None and r_p_mid is not None:
            m_angle = cal_angle(r_p_mid[0] - self.line_width//2, 0)
            m_curve = r_curve
            m_dis = r_dis
            m_far = (r_p_bot[0] - self.line_width//2) - 160
        
        return [l_angle, l_curve, l_dis, l_far], [m_angle, m_curve, m_dis, m_far], [r_angle, r_curve, r_dis, r_far]

    def detect(self, img):
        preImg = self.preProcess.go(img)
        birdview_rgb = None 
        if self.debug:
            birdview_rgb = cv2.cvtColor(preImg, cv2.COLOR_GRAY2RGB)
        # filter solid and dotted lane from binary image
        dotted_cnts, solid_cnts, solid_dotted_cnts = self.get_dotted_solid_contours(preImg)
        solid_dotted_mask = self.create_mask_line(solid_dotted_cnts)
        # extract only dotted lane on a mask
        # dotted_mask = self.create_mask_line(dotted_cnts)
        dotted_line = self.find_dotted_line(birdview_rgb, dotted_cnts)

        solid_mask = self.create_mask_line(solid_cnts)
        
        solid_left, solid_right = self.find_solid_line(solid_mask, solid_cnts)
        if self.debug:
            for dot in dotted_cnts:
                rect = cv2.minAreaRect(dot)
                box = np.int0(cv2.boxPoints(rect))
                cv2.drawContours(birdview_rgb,[box],0,(0,255,0),1)

            for sol in solid_cnts:
                rect = cv2.minAreaRect(sol)
                box = np.int0(cv2.boxPoints(rect))
                cv2.drawContours(birdview_rgb,[box],0,(255,255,0),1)
            cv2.imshow("solid mask", solid_mask)
            cv2.imshow("pre_img", preImg)
            cv2.imshow("birdview_rgb", birdview_rgb)
            cv2.imshow("solid_dotted_mask", solid_dotted_mask)
            cv2.waitKey(1)
        
        return self.calc_line_properties(solid_left, solid_right, dotted_line)

