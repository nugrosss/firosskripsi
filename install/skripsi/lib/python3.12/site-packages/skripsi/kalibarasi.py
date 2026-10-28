import numpy as np


# Data hasil pengukuran
# Ganti dengan data asli kamera kamu
jarak_deteksi = np.array([
    10.4,  # 12 cm
    11.2,  # 13 cm
    11.6,  # 14 cm
    12.5,  # 15 cm
    16.4,  # 20 cm
    20.41,    # 25 cm
    26.4,    # 30 cm
    31.5,    # 35 cm
    36.66,    # 40 cm
    41.5,    # 45 cm
    46.7,    # 50 cm
    50,    # 55 cm
    55,    # 60 cm
    60,    # 65 cm
    63,    # 70 cm
    68,    # 75 cm
    72,    # 80 cm
    77,    # 85 cm
    80,    # 90 cm
    85,    # 95 cm
    89,    # 100 cm
    92,     # 105 cm
    95,
], dtype=float)

jarak_asli = np.array([
    12,     # 12 cm
    13,     # 13 cm
    14,     # 14 cm
    15,     # 15 cm
    20,     # 20 cm
    25,     # 25 cm
    30,     # 30 cm
    35,     # 35 cm
    40,     # 40 cm
    45,     # 45 cm
    50,     # 50 cm
    55,     # 55 cm
    60,     # 60 cm
    65,     # 65 cm
    70,     # 70 cm
    75,     # 75 cm
    80,     # 80 cm
    85,     # 85 cm
    90,
    95,  # 90.95 cm
    100,    # 100 cm
    105,    # 105 cm
    110     # 110 cm
], dtype=float)

# Regresi polinomial derajat 2
koefisien = np.polyfit(
    jarak_deteksi,
    jarak_asli,
    deg=2
)

model = np.poly1d(koefisien)

print("Persamaan koreksi:")
print(model)


def koreksi_jarak(jarak):
    return float(model(jarak))


# Contoh pengujian
for jarak in [15, 30, 45, 50, 60, 75, 100]:
    hasil = koreksi_jarak(jarak)

    print(
        f"Deteksi: {jarak:6.2f} cm | "
        f"Koreksi: {hasil:6.2f} cm"
    )