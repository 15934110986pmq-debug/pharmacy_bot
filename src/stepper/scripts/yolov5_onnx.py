import onnxruntime
import numpy as np
import cv2
import os
import glob

class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))

colors = Colors()  # create instance for 'from utils.plots import colors'

def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLov5 project.
    param: 
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    return:
        no return

    """
    tl = (
        line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )
class YOLOV5():
    def __init__(self,onnxpath, classes, conf_thresh=0.8, iou_threshold=0.4):
        self.onnx_session=onnxruntime.InferenceSession(onnxpath)
        self.input_name=self.get_input_name()
        self.output_name=self.get_output_name()
        self.CONF_THRESH = conf_thresh
        self.IOU_THRESHOLD = iou_threshold
        self.classes = classes

    def get_input_name(self):
        input_name=[]
        for node in self.onnx_session.get_inputs():
            input_name.append(node.name)
        return input_name

    def get_output_name(self):
        output_name=[]
        for node in self.onnx_session.get_outputs():
            output_name.append(node.name)
        return output_name

    def get_input_feed(self,img_tensor):
        input_feed={}
        for name in self.input_name:
            input_feed[name]=img_tensor
        return input_feed

    def inference(self,img):
        # 进行resize
        or_img = cv2.resize(img, (640, 640))
        # BGR2RGB和HWC2CHW
        img = or_img[:, :, ::-1].transpose(2, 0, 1)
        img = img.astype(dtype=np.float32)
        img /= 255.0
        img = np.expand_dims(img, axis=0)
        # 准备模型输入和推理
        input_feed = self.get_input_feed(img)
        pred = self.onnx_session.run(None, input_feed)[0]
        
        box_data = self.filter_box(pred, self.CONF_THRESH, self.IOU_THRESHOLD)
        if box_data.size == 0:
            boxes = []
            scores = []
            classid = []
        else:   

            boxes = box_data[..., :4].astype(np.int32)
            scores = box_data[..., 4]
            classid = box_data[..., 5].astype(np.int32)
            for box, cls_conf, cls_id in zip(boxes, scores, classid):

                color = colors(cls_id, True)
                plot_one_box(
                    box,
                    or_img,
                    color=color,
                    label="{}:{:.2f}".format(
                    self.classes[cls_id], cls_conf
                    ),
                )
        return boxes, scores, classid

    def xywh2xyxy(self,x):
        y = np.copy(x)
        input_w=640
        input_h=640
        origin_w=640
        origin_h=480
        r_w = input_w / origin_w
        r_h = input_h / origin_h

        y[:, 0] = x[:, 0] - x[:, 2] / 2
        y[:, 1] = x[:, 1] - x[:, 3] / 2- (input_h - r_w * origin_h) / 2
        y[:, 2] = x[:, 0] + x[:, 2] / 2
        y[:, 3] = x[:, 1] + x[:, 3] / 2 - (input_h - r_w * origin_h) / 2

        return y


    def nms(self,dets, conf_thresh):
        x1 = dets[:, 0]
        y1 = dets[:, 1]
        x2 = dets[:, 2]
        y2 = dets[:, 3]
        areas = (y2 - y1 + 1) * (x2 - x1 + 1)
        scores = dets[:, 4]
        keep = []
        index = scores.argsort()[::-1]
        while index.size > 0:
            i = index[0]
            keep.append(i)
            x11 = np.maximum(x1[i], x1[index[1:]])
            y11 = np.maximum(y1[i], y1[index[1:]])
            x22 = np.minimum(x2[i], x2[index[1:]])
            y22 = np.minimum(y2[i], y2[index[1:]])
            w = np.maximum(0, x22 - x11 + 1)
            h = np.maximum(0, y22 - y11 + 1)
            overlaps = w * h
            ious = overlaps / (areas[i] + areas[index[1:]] - overlaps)
            idx = np.where(ious <= conf_thresh)[0]
            index = index[idx + 1]
        return keep

    def filter_box(self,org_box, conf_thres, iou_threshold):
        org_box = np.squeeze(org_box)
        conf = org_box[..., 4] > conf_thres
        box = org_box[conf == True]
        cls_cinf = box[..., 5:]
        cls = []
        for i in range(len(cls_cinf)):
            cls.append(int(np.argmax(cls_cinf[i])))
        all_cls = list(set(cls))
        output = []
        for i in range(len(all_cls)):
            curr_cls = all_cls[i]
            curr_cls_box = []
            curr_out_box = []
            for j in range(len(cls)):
                if cls[j] == curr_cls:
                    box[j][5] = curr_cls
                    curr_cls_box.append(box[j][:6])
            curr_cls_box = np.array(curr_cls_box)
            curr_cls_box = self.xywh2xyxy(curr_cls_box)
            curr_out_box = self.nms(curr_cls_box, iou_threshold)
            for k in curr_out_box:
                output.append(curr_cls_box[k])
        output = np.array(output)
        return output
    
        
if __name__ == "__main__":
    onnx_path='/home/ubuntu/weights/garbage_classification/garbage_classification.onnx'
    classes= ['BananaPeel', 'BrokenBones', 'CigaretteEnd', 'DisposableChopsticks', 'Ketchup', 'Marker', 'OralLiquidBottle', 'Plate', 'PlasticBottle', 'StorageBattery', 'Toothbrush', 'Umbrella']
    YoLov5 = YOLOV5(onnx_path,classes, 0.85, 0.5)


    # 打开摄像头
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        boxes, scores, classid = YoLov5.inference(frame)
        for box, cls_conf, cls_id in zip(boxes, scores, classid):
    
            color = colors(cls_id, True)
            plot_one_box(
                box,
                frame,
                color=color,
                label="{}:{:.2f}".format(
                classes[cls_id], cls_conf
                ),
            )
    
        cv2.imshow('Detected Video', frame)

        # 按下 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

