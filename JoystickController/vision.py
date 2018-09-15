import cv2
import picamera
import picamera.array
import time

orb = cv2.ORB_create(scaleFactor = 1.5)

with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (320, 240)
        camera.vflip = True
        camera.hflip = True

        while True:
            start_time = time.time()
            camera.capture(stream, 'bgr', use_video_port=True)
            # stream.array now contains the image data in BGR order
            kp = orb.detect(stream.array)
            t1 = time.time() - start_time
            kp, des = orb.compute(stream.array, kp)
            t2 = time.time() - start_time - t1
            img = stream.array
            cv2.drawKeypoints(stream.array, kp, img, color = (0, 255, 0), flags = 0)
            cv2.imshow('frame', img)
            t3 = time.time() - start_time -t2 - t1
            print((t1,t2,t3))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # reset the stream before the next capture
            stream.seek(0)
            stream.truncate()

        cv2.destroyAllWindows()



