#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Run YOLOv5 detection inference on images, videos, directories, globs, YouTube, webcam, streams, etc.

Usage - sources:
    $ python detect.py --weights yolov5s.pt --source 0                               # webcam
                                                     img.jpg                         # image
                                                     vid.mp4                         # video
                                                     screen                          # screenshot
                                                     path/                           # directory
                                                     list.txt                        # list of images
                                                     list.streams                    # list of streams
                                                     'path/*.jpg'                    # glob
                                                     'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                     'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python detect.py --weights yolov5s.pt                 # PyTorch
                                 yolov5s.torchscript        # TorchScript
                                 yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                 yolov5s_openvino_model     # OpenVINO
                                 yolov5s.engine             # TensorRT
                                 yolov5s.mlmodel            # CoreML (macOS-only)
                                 yolov5s_saved_model        # TensorFlow SavedModel
                                 yolov5s.pb                 # TensorFlow GraphDef
                                 yolov5s.tflite             # TensorFlow Lite
                                 yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
                                 yolov5s_paddle_model       # PaddlePaddle
"""
import rospy
import argparse
import os
import platform
import sys
from pathlib import Path

import torch

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

print(type(ROOT))

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode

from carry_my_luggage.msg import PaperDetect

    # p = PaperDetect()    # p = PaperDetect()
                            # p.robo_p_dis = cam_pprbg_dis
                            # p.robo_p_drct = cam_pprbg_drct #改良してから変更する
                            # # p.p_exist = person_count
                            # paper_pub.publish(p)
                            # p.robo_p_dis = cam_pprbg_dis
                            # p.robo_p_drct = cam_pprbg_drct #改良してから変更する
                            # # p.p_exist = person_count
                            # paper_pub.publish(p)
@smart_inference_mode()

class BagDetect():
    def __init__(self):
        self.paper_pub = rospy.Publisher("/paper", PaperDetect, queue_size=1)

    def run(self,
            weights=ROOT / 'yolov5s.pt',  # model path or triton URL
            source=ROOT / 'data/',  # file/dir/URL/glob/screen/0(webcam)
            #data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
            data=ROOT / 'data/data2.yaml',
            imgsz=(640, 640),  # inference size (height, width)
            conf_thres=0.25,  # confidence threshold
            iou_thres=0.45,  # NMS IOU threshold
            max_det=1000,  # maximum detections per image
            device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
            view_img=False,  # show results
            save_txt=False,  # save results to *.txt
            save_conf=False,  # save confidences in --save-txt labels
            save_crop=False,  # save cropped prediction boxes
            nosave=False,  # do not save images/videos
            classes=None,  # filter by class: --class 0, or --class 0 2 3
            agnostic_nms=False,  # class-agnostic NMS
            augment=False,  # augmented inference
            visualize=False,  # visualize features
            update=False,  # update all models
            project=ROOT / 'runs/detect',  # save results to project/name
            name='exp',  # save results to project/name
            exist_ok=False,  # existing project/name ok, do not increment
            line_thickness=3,  # bounding box thickness (pixels)
            hide_labels=False,  # hide labels
            hide_conf=False,  # hide confidences
            half=False,  # use FP16 half-precision inference
            dnn=False,  # use OpenCV DNN for ONNX inference
            vid_stride=1,  # video frame-rate stride
    ):
        #画像の幅と高さを保持する変数
        im_height = 0
        im_width = 0
        get_count = 0 #1回だけ画像の幅と高さを取得する変数

        cam_pprbg_dis = 3 #紙袋とカメラとの距離を保持する
        cam_pprbg_drct = 3 #カメラから見た紙袋の方向を保持する

        """
        紙袋の距離と方向の出版者を作る。
        """

        paper_pub = rospy.Publisher("/paper", PaperDetect, queue_size=1)



        source = str(source)
        save_img = not nosave and not source.endswith('.txt')  # save inference images
        is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
        is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
        webcam = source.isnumeric() or source.endswith('.streams') or (is_url and not is_file)
        screenshot = source.lower().startswith('screen')
        if is_url and is_file:
            source = check_file(source)  # download

        # Directories
        save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
        (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

        # Load model
        device = select_device(device)
        model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
        stride, names, pt = model.stride, model.names, model.pt
        imgsz = check_img_size(imgsz, s=stride)  # check image size

        # Dataloader
        bs = 1  # batch_size
        if webcam:
            # view_img = check_imshow(warn=True)
            view_img = False
            dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
            bs = len(dataset)
        elif screenshot:
            dataset = LoadScreenshots(source, img_size=imgsz, stride=stride, auto=pt)
        else:
            dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
        vid_path, vid_writer = [None] * bs, [None] * bs

        # Run inference
        model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
        seen, windows, dt = 0, [], (Profile(), Profile(), Profile())

        #ここが繰り替えされる部分
        for path, im, im0s, vid_cap, s in dataset:

            #1回だけ画像の幅と高さを取得する
            if get_count == 0:
                im_height, im_width = im.shape[2:]
                get_count = 1

            #print("リアルタイムで動いている")

            with dt[0]:
                im = torch.from_numpy(im).to(model.device)
                im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
                im /= 255  # 0 - 255 to 0.0 - 1.0
                if len(im.shape) == 3:
                    im = im[None]  # expand for batch dim

            # Inference
            with dt[1]:
                visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
                pred = model(im, augment=augment, visualize=visualize)

            # NMS
            with dt[2]:
                pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

            # Second-stage classifier (optional)
            # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

            # Process predictions
            for i, det in enumerate(pred):  # per image
                seen += 1
                if webcam:  # batch_size >= 1
                    p, im0, frame = path[i], im0s[i].copy(), dataset.count
                    s += f'{i}: '
                else:
                    p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

                p = Path(p)  # to Path
                save_path = str(save_dir / p.name)  # im.jpg
                txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
                s += '%gx%g ' % im.shape[2:]  # print string
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
                imc = im0.copy() if save_crop else im0  # for save_crop
                annotator = Annotator(im0, line_width=line_thickness, example=str(names))


                print("\n\n")

                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                    #print(det)#検出結果を表示する

                    # Print results
                    for c in det[:, 5].unique():
                        n = (det[:, 5] == c).sum()  # detections per class
                        s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                    # Write results
                    #リストのアンパックをしている。(開いて中の要素を取得する)
                    #reserved関数でリストの逆順を表示している

                    #print("len(det)=" + str(len(det)))

                    index = 0 #添字番号を保持する

                    #print(det)

                    #紙袋もとっても検出されていないとき
                    if len(det) == 0:
                        cam_pprbg_dis = 3
                        cam_pprbg_drct = 3
                        print("紙袋が見つかりません。")
                        #探す

                
                    #紙袋かとってのいづれかが検出されたとき
                    else:
                        for *xyxy, conf, cls in reversed(det):
                            if save_txt:  # Write to file
                                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                                line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                                with open(f'{txt_path}.txt', 'a') as f:
                                    f.write(('%g ' * len(line)).rstrip() % line + '\n')

                            if save_img or save_crop or view_img:  # Add bbox to image
                                c = int(cls)  # integer class
                                label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                                annotator.box_label(xyxy, label, color=colors(c, True))

                        
                            #print("画像の高さ=" + str(im_height))
                            #print("画像の幅=" + str(im_width))

                            w = int(xyxy[2]) - int(xyxy[0])
                            h = int(xyxy[3]) - int(xyxy[1])
                
                            c_x = int((int(xyxy[0]) + int(xyxy[2])) / 2)
                            c_y = int((int(xyxy[1]) + int(xyxy[3])) / 2)

                            #print("idx=" + str(index+1))
                            #print("xyxy[0]=" + str(int(xyxy[0])))

                            #クラスが紙袋のとき
                            if cls == 0:
                                #print("左上=(" + str(int(xyxy[0])) + ", " + str(int(xyxy[1])) + ")")
                                #print("右下=(" + str(int(xyxy[2])) + ", " + str(int(xyxy[3])) + ")")

                                #print("幅=" + str(w))
                                #print("高さ=" + str(h))
                                #print("矩形の中心=(" + str(c_x) + ", " + str(c_y) + ")")

                                #print("幅=")
                                
                                #print("クラス=" + str(int(cls)))
                                #print("idx=" + str(index)) #添字番号
                                #print("\n")
                                index+=1 #添字番号を増やす

                                #紙袋が近い
                                if w > 400:
                                    print("紙袋が近い")
                                    cam_pprbg_dis = 2

                                #紙袋が中央
                                elif w >= 320 and w <= 400:
                                    print("紙袋が中央にある")
                                    cam_pprbg_dis = 1

                                #紙袋が遠い
                                elif w < 320:
                                    print("紙袋が遠い")
                                    cam_pprbg_dis = 0


                                #紙袋が画面左にある
                                if c_x > im_width*2/3 :
                                    print("紙袋がカメラから見て画面左にある")
                                    cam_pprbg_drct = 0

                                #紙袋が画面中央
                                elif c_x >= im_width/3 and w <= im_width*2/3:
                                    print("紙袋がカメラから見て画面中央にある")
                                    cam_pprbg_drct = 1

                                #紙袋が画面右にある
                                elif c_x < im_width/3:
                                    print("紙袋がカメラから見て画面右にある")
                                    cam_pprbg_drct = 2


                                p = PaperDetect()
                                p.robo_p_dis = cam_pprbg_dis
                                p.robo_p_drct = cam_pprbg_drct 
                                self.paper_pub.publish(p)


                            #print(xyxy) #ここにバウンディングボックスの座標が入っている。
                            #print("conf=" + str(conf)) #ここにその物体である確率が入っている。
                            #print("cls=" + str(cls)) #ここにラベリングのクラスが入っている。


                            if save_crop:
                                save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)

                print("紙袋の距離と方向")
                print("cam_pprbg_dis=" + str(cam_pprbg_dis))
                print("cam_pprbg_drct=" + str(cam_pprbg_drct))


                # Stream results
                im0 = annotator.result()
                if view_img:
                    #print("画面に表示します。") #ここで画面のキャプチャを表示する
                    if platform.system() == 'Linux' and p not in windows:
                        windows.append(p)
                        cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                        cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                    cv2.imshow(str(p), im0)
                    cv2.waitKey(1)  # 1 millisecond

                # Save results (image with detections)
                if save_img:
                    if dataset.mode == 'image':
                        cv2.imwrite(save_path, im0)
                    else:  # 'video' or 'stream'
                        if vid_path[i] != save_path:  # new video
                            vid_path[i] = save_path
                            if isinstance(vid_writer[i], cv2.VideoWriter):
                                vid_writer[i].release()  # release previous video writer
                            if vid_cap:  # video
                                fps = vid_cap.get(cv2.CAP_PROP_FPS)
                                w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                                h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            else:  # stream
                                fps, w, h = 30, im0.shape[1], im0.shape[0]
                            save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
                            vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                        vid_writer[i].write(im0)

            #print("type(s)=" + str(type(s)))
            #print("s=" + str(s))
            # Print time (inference-only)
            LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")

        # Print results
        t = tuple(x.t / seen * 1E3 for x in dt)  # speeds per image
        LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
        if save_txt or save_img:
            s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
            LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
        if update:
            strip_optimizer(weights[0])  # update model (to fix SourceChangeWarning)


    def parse_opt(self):
        parser = argparse.ArgumentParser()
        #parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'yolov5s.pt', help='model path or triton URL')
        parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'models/pprbg_dtcr.pt', help='model path or triton URL')
        #parser.add_argument('--source', type=str, default=ROOT / 'data/images', help='file/dir/URL/glob/screen/0(webcam)')
        parser.add_argument('--source', type=str, default=0, help='file/dir/URL/glob/screen/0(webcam)')
        parser.add_argument('--data', type=str, default=ROOT / 'data/coco128.yaml', help='(optional) dataset.yaml path')
        parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
        #parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
        parser.add_argument('--conf-thres', type=float, default=0.35, help='confidence threshold')
        parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
        parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
        parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
        parser.add_argument('--view-img', action='store_true', help='show results')
        parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
        parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
        parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
        parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
        parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
        parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
        parser.add_argument('--augment', action='store_true', help='augmented inference')
        parser.add_argument('--visualize', action='store_true', help='visualize features')
        parser.add_argument('--update', action='store_true', help='update all models')
        parser.add_argument('--project', default=ROOT / 'runs/detect', help='save results to project/name')
        #parser.add_argument('--name', default='exp', help='save results to project/name')
        parser.add_argument('--name', default='demo', help='save results to project/name')
        parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
        parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
        parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
        parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
        parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
        parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
        parser.add_argument('--vid-stride', type=int, default=1, help='video frame-rate stride')
        opt, unknown = parser.parse_known_args()
        #opt, unknown = parser.parse_args()
        #print("opt=" + str(opt))
        #print("unknown=" + str(unknown))


        opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
        print_args(vars(opt))
        return opt


    def main(self, opt):
        check_requirements(exclude=('tensorboard', 'thop'))
        self.run(**vars(opt))


if __name__ == "__main__":
    rospy.init_node("pprbgdtc")
    paper_detect = BagDetect()
    opt = paper_detect.parse_opt()

    while not rospy.is_shutdown():
        paper_detect.main(opt)
        rospy.Rate(10).sleep()