jarak_deteksi = 14

        # Masukkan hasil servo ke fungsi koreksi
jarak_koreksi = jarak_deteksi
if jarak_deteksi < 15: 
    jarak_koreksi += 1
elif jarak_koreksi > 15:
    jarak_koreksi -1

print(jarak_koreksi)