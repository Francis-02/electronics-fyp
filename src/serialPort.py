import serial
from datetime import datetime

PUERTO = "COM11"      # Cambia esto
BAUDIOS = 115200

nombre_archivo = datetime.now().strftime("datos_%Y%m%d_%H%M%S.txt")

ser = serial.Serial(PUERTO, BAUDIOS, timeout=1)

print(f"Guardando en {nombre_archivo}")
print("Esperando datos...\n")

try:
    with open(nombre_archivo, "a") as archivo:
        while True:
            linea = ser.readline().decode("utf-8", errors="ignore").strip()

            if linea:
                print(linea)          # Mostrar por consola
                archivo.write(linea + "\n")
                archivo.flush()       # Guardar inmediatamente

except KeyboardInterrupt:
    print("\nCaptura detenida.")

finally:
    ser.close()