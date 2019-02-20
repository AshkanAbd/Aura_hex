import cv2 as cv

print(cv.__version__)
exit(0)
def process():
    capture = cv.VideoCapture(0)
    while True:
        _, frame = capture.read()
        frame = cv.flip(frame, 1)
        frame_blur = cv.GaussianBlur(frame, (5, 5), -1)
        frame_edge = cv.Canny(frame_blur, 100, 200)
        _, contours, _ = cv.findContours(frame_edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            esp = 0.02 * cv.arcLength(cnt, True)
            approx = cv.approxPolyDP(cnt, esp, True)
            cv.drawContours(frame, [cnt], -1, (0, 0, 255))
            cv.putText(frame, str(len(approx)), (approx[0, 0, 0] - 2, approx[0, 0, 1] - 2), 1, 1, (255, 0, 0))

        cv.imshow('a', frame)
        cv.imshow('b', frame_edge)
        if cv.waitKey(1) == 27: break


if __name__ == '__main__':
    process()
