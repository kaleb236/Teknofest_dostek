from pyzbar import pyzbar

import cv2

cap = cv2.VideoCapture('/dev/video0')

while True:
    ret, frame = cap.read()
    barcodes = pyzbar.decode(frame)

    for barcode in barcodes:
        (x, y, w, h) = barcode.rect

        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    cv2.imshow('image', frame)
    cv2.waitKey(1)