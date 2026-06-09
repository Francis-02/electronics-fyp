import os
import glob
import pandas as pd
from scipy.io import savemat

# Carpeta donde están los CSVs
carpeta = "."  # "." es la carpeta actual
files = sorted(glob.glob(os.path.join(carpeta, "registro_step_*.csv")))

if not files:
    print("No se encontraron archivos CSV")
    exit()

# Diccionario que se exportará a MATLAB
mat_data = {}

for fichero in files:
    print(f"Procesando {fichero}...")
    
    try:
        # Leer CSV
        df = pd.read_csv(fichero, header=None, names=["tiempo","angulo","step"])
    except Exception as e:
        print(f"Error al leer {fichero}: {e}")
        continue
    
    # Convertir todo a numérico y eliminar filas no válidas
    df = df.apply(pd.to_numeric, errors='coerce').dropna()
    if df.empty:
        print(f"{fichero} no tiene datos válidos")
        continue
    
    # Ordenar por tiempo
    df = df.sort_values("tiempo").reset_index(drop=True)
    
    # Calcular velocidad (derivada)
    df["velocidad"] = df["angulo"].diff() / df["tiempo"].diff()
    
    # Guardar como array para MATLAB
    nombre_var = os.path.basename(fichero).replace(".csv","")
    mat_data[nombre_var] = {
        "tiempo": df["tiempo"].to_numpy(),
        "angulo": df["angulo"].to_numpy(),
        "step": df["step"].to_numpy(),
        "velocidad": df["velocidad"].fillna(0).to_numpy()  # NaN -> 0
    }

# Guardar todos los datos en un único archivo .mat
archivo_mat = "todos_los_registros.mat"
savemat(archivo_mat, mat_data)
print(f"Datos guardados en {archivo_mat}")
