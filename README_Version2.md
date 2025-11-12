# HEDS-5540 / TB6600 / Arduino Nano - Paket

Bu paket aşağıdaki dosyaları içerir:
- HEDS5540_Arduino.ino : Genel quadrature örneği
- HEDS5540_Nano.ino : Nano için test sketch (CHANGE)
- HEDS5540_Nano_fast.ino : Tek-kanal RISING (performans için)
- measure_with_index.ino : Z indeksi varsa otomatik PPR ölçümü
- measure_manual_one_rev.ino : Elle 1 tur ölçümü (Z yoksa)
- closed_loop_simple_nano.ino : Basit closed-loop (düşük hız/test)
- closed_loop_nano_tb6600.ino : Nano-only closed-loop + Timer1 step üretimi (daha ileri)

Önemli notlar:
- Her dosyada açıklamalar ve hangi pinlerin kullanıldığı yazılıdır. Lütfen gerçek donanımınıza göre pinleri/girişleri doğrulayın.
- Enkoder pull-up: datasheet 2.7 kΩ öneriyor. Harici 2.7k kullanın.
- Enkoder_counts_per_rev değerini (quadrature kenar sayısı/rev) ölçüp `closed_loop_nano_tb6600.ino` içindeki `encoder_counts_per_rev` değerini güncelleyin.
- TB6600 step pulse minimum genişliğini kontrol edin. Donanım toggle kullanan Timer1, pulse genişliğini frekansa göre değiştirir; gerektiğinde Timer ISR ile sabit pulse width oluşturun.
- Bu paket Nano ile düşük/orta hız testleri için uygundur. Yüksek performans için donanım sayaç (LS7366R) veya daha güçlü MCU önerilir.

ZIP oluşturma (Linux / macOS / WSL)
1. bir dizin oluşturun:
   mkdir heds5540_package
   cd heds5540_package
2. her dosyayı yukarıdaki içeriklerle kaydedin (ör. HEDS5540_Arduino.ino).
3. zip oluşturun:
   zip -r ../heds5540_package.zip .

Windows (PowerShell)
1. klasör oluşturun ve dosyaları içine kaydedin (örn. `heds5540_package`).
2. PowerShell ile:
   Compress-Archive -Path .\heds5540_package\* -DestinationPath .\heds5540_package.zip

Eğer isterseniz:
- Ben bu dosyaları bir GitHub repo'suna push edebilirim (sizin repo'nuzsa veya yeni bir repo oluşturmak isterseniz). Repo bilgisi verirseniz push için bir branch oluşturup dosyaları gönderebilirim.
- Ya da size doğrudan Measurement sonucu gönderin (ör. quadrature counts/rev), ben `encoder_counts_per_rev` değerini güncelleyip `closed_loop_nano_tb6600.ino`'yu size hazır şekilde revize ederim.