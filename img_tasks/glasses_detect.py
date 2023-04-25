import cv2
import torch


def get_dis_drct():

    """
    pushlisher作る。
    """

    #model = torch.hub.load('ultralytics/yolov5', 'yolov5s') 
    #Qiita, yolov5のモデルをオフラインで使用する, https://qiita.com/Decwest/items/6ef2383787baa7b83143, 2023年3月19日.
    #yolov5のパスは絶対、モデルのパスは相対で指定した。
    model = torch.hub.load('/home/ri-one/Desktop/github_local_repository/yolov5', 'custom', path='19sbest_glasses.pt', source='local')
    #model = torch.hub.load('/home/ri-one/Desktop/github_local_repository/yolov5', 'custom', path='22sbest_pprbg.pt', source='local')
    model.conf = 0.5
    
    #print(model.names)

    cap = cv2.VideoCapture(0)
    ret, img= cap.read() #画像の大きさを取得するために1度だけ最初によびだす。

    #人が写っていない前提で初期化する
    robo_p_dis = 3 #ロボットと人との距離感覚
    robo_p_drct = 3 #ロボットと人との方向感覚

    c = 0
    heigh = 0 #カメラから取得した画像の高さを保持
    width = 0 #カメラから取得した画像の幅を保持

    glasses_count = 0 #メガネが検出されたかどうか
    put_glss = 0 #最終的にその人がメガネをかけているかを判定する

    while True:
        ret, img= cap.read()
        result = model(img)

        if c == 0:
            height, width, _ = img.shape[:3]
            print("高さ=" + str(height))
            print("幅=" + str(width ))
            c += 1
        

        #推論結果を取得
        obj = result.pandas().xyxy[0]

        
        #人が写っているかを調べる
        for i in range(len(obj)):
            if obj.name[i] == "glasses":
                glasses_count += 1
                break

        
        
        #メガネが写っていないとき
        if glasses_count == 0:

            #メガネをかけていない。
            put_glss = 0



        #メガネが写っているとき
        if glasses_count == 1:

            glss_w_l = [] #検出されたメガネの大きさを保持
            glss_idx_l = [] #検出されたメガネの添字を保持

            #バウンディングボックスの情報を取得
            for  i in range(len(obj)):
                
                #検出された物体がメガネのときに
                if obj.name[i] == "glasses":

                    #人のときだけ計算することで無駄な計算を削減する。
                    xmin = obj.xmin[i]
                    ymin = obj.ymin[i]
                    xmax = obj.xmax[i]
                    ymax = obj.ymax[i]

                    glss_w_l.append(xmax-xmin) #幅を計算して追加する
                    glss_idx_l.append(i) #添字を追加する

                    #print("name =", name, "xmin =", xmin, "ymin =", ymin, "xmax =", ymax, "ymin =", ymax)

                
                #print("(x_min=" + str(xmin) + ", y_min=" + str(ymin) + ")" + "(x_max=" + str(xmax) + ", y_max=" + str(ymax) + ")")
                    
                #print("名前" + str(name) + "幅:" + str(w) + ", 高さ:" + str(h))

            #一番近くにあるメガネを検出する
            print("glss_w_l=" + str(glss_w_l))
            print("glss_idx_l=" + str(glss_idx_l))
            print("max(glss_w_l)=" + str(max(glss_w_l)))
            max_w_idx = glss_w_l.index(max(glss_w_l))
            print("glss_idx_l[max_w_idx]=" + str(glss_idx_l[max_w_idx]))



            #一番近くにあるメガネのみを判定に掛ける
            xmin = obj.xmin[glss_idx_l[max_w_idx]]
            ymin = obj.ymin[glss_idx_l[max_w_idx]]
            xmax = obj.xmax[glss_idx_l[max_w_idx]]
            ymax = obj.ymax[glss_idx_l[max_w_idx]]

            #print("name =", name, "xmin =", xmin, "ymin =", ymin, "xmax =", ymax, "ymin =", ymax)

            w = xmax - xmin #矩形の幅
            h = ymax - ymin #矩形の高さ
            c_x = (xmax + xmin)/2 #矩形の中心のx座標
            c_y = (ymax + ymin)/2 #矩形の中心のy座標


            if w < 350:
                #print(str(i) + "番目の人が遠い")
                robo_p_dis = 0 #ロボットは人が中央に来るまで前に進む

            elif w >= 350 and w <= 550:
                #print(str(i) + "番目の人が中央の距離")
                robo_p_dis = 1 #ロボットはそのまま

            elif w > 550:
                #print(str(i) + "番目の人が近い")
                robo_p_dis = 2 #ロボットは人が中央に来るまで後ろに下がる

            if c_x < width/3:
                #print(str(i) + "番目の人が左にいる")
                robo_p_drct = 0 #ロボットは人が中央に来るまで左回りする

            elif c_x > width/3 and c_x < width * 2/3:
                #print(str(i) + "番目の人が中央の方向")
                robo_p_drct = 1 #ロボットはそのまま
                put_glss = 1

            elif c_x > width * 2/3:
                #print(str(i) + "番目の人が右にいる")
                robo_p_drct = 2 #ロボットは人が中央に来るまで右回りする
                
            print("距離:" + str(robo_p_dis) + "、方向:" + str(robo_p_drct))
            print("メガネをかけている=" + str(put_glss))

        glasses_count = 0 #判定するための変数を初期化する
        put_glss = 0 #メガネの有無の変数の初期化

        #print("距離=" + str(robo_p_dis))
        #print("方向=" + str(robo_p_drct))

        """
        ここで、距離と方向をPublishしてほしい。
        """

        #バウンディングボックスを描画
        result.render()
        cv2.imshow('result', result.ims[0])

        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    
if __name__ == "__main__":
    get_dis_drct()