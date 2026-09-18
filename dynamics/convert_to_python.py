import os
import shutil
import tkinter as tk
from tkinter import filedialog
from tqdm import tqdm

# --- CONFIGURACIÓN DE REEMPLAZOS ---
# Diccionario: "Texto a buscar": "Texto nuevo"
# Para borrar algo, usa cadena vacía "".
REGLAS = {
    ".*": "*",   # Corregir mult
    ".^": "**",  # Corregir power
    "cos(": "np.cos(",
    "sin(": "np.sin(",
    # "atan(": "np.arctan(",
    "tan(": "np.tan(",
    ";\n": "\n",
    "./": "/",
    "np.np.np.": "np.",
    "np.np.": "np."     
}

def procesar_texto(archivo_entrada):
    # Creamos un nombre para el archivo temporal
    archivo_temp = archivo_entrada + ".tmp_processing"
    
    file_size = os.path.getsize(archivo_entrada)
    
    print(f"Procesando: {os.path.basename(archivo_entrada)}")
    
    try:
        # Abrimos origen (lectura) y temp (escritura)
        # 'utf-8' es estándar, pero si te da error cambia a 'latin-1'
        with open(archivo_entrada, 'r', encoding='utf-8', errors='ignore') as f_in, \
             open(archivo_temp, 'w', encoding='utf-8') as f_out:
            
            # Barra de progreso basada en BYTES leídos
            with tqdm(total=file_size, unit='B', unit_scale=True, desc="Reemplazando") as pbar:
                
                for linea in f_in:
                    nueva_linea = linea
                    
                    # Aplicar todas las reglas del diccionario a esta línea
                    for viejo, nuevo in REGLAS.items():
                        if viejo in nueva_linea:
                            nueva_linea = nueva_linea.replace(viejo, nuevo)
                    
                    f_out.write(nueva_linea)
                    
                    # Actualizamos la barra según lo que leímos (longitud en bytes)
                    pbar.update(len(linea.encode('utf-8')))

        # INTERCAMBIO ATÓMICO
        # Solo si todo salió bien, borramos el original y renombramos el temporal
        shutil.move(archivo_temp, archivo_entrada)
        print("Éxito: Archivo modificado correctamente.")

    except Exception as e:
        print(f"\nError crítico: {e}")
        if os.path.exists(archivo_temp):
            os.remove(archivo_temp) # Limpieza si falló
            print("Se eliminó el archivo temporal. El original está intacto.")

# --- UI ---
if __name__ == "__main__":
    root = tk.Tk()
    root.withdraw()
    ruta_archivo = filedialog.askopenfilename(title="Seleccionar archivo de texto (.txt, .csv, .log, .sql)")
    root.destroy()

    if ruta_archivo:
        procesar_texto(ruta_archivo)
