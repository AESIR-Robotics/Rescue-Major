import cv2

cap = cv2.VideoCapture(1)  # Usa tu cámara (o cambia a un archivo de video)

ret, frame1 = cap.read()
ret, frame2 = cap.read()

while cap.isOpened():
    # Calcular diferencia entre dos cuadros consecutivos
    diff = cv2.absdiff(frame1, frame2)
    
    # Convertir a escala de grises
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    
    # Suavizado para reducir ruido
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Umbral para extraer las zonas en movimiento
    _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
    
    # Dilatación para rellenar huecos
    dilated = cv2.dilate(thresh, None, iterations=3)
    
    # Encontrar contornos (zonas en movimiento)
    contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Dibujar rectángulos donde se detecta movimiento
    for contour in contours:
        if cv2.contourArea(contour) < 10000:
            continue  # Ignorar áreas pequeñas
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame1, (x, y), (x+w, y+h), (0, 255, 0), 2)

    cv2.imshow("Movimiento", frame1)

    frame1 = frame2
    ret, frame2 = cap.read()

    if cv2.waitKey(10) == 27:
        break

cap.release()
cv2.destroyAllWindows()
