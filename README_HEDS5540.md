# HEDS-5540 / TB6600 / Arduino Nano - README

Bu README, proje örnekleri, ölçüm sonuçları ve hızlı başlangıç talimatlarını içerir.

## Önemli Ölçüm
- Measured encoder CPR: 400
- Quadrature counts/rev = CPR * 4 = 1600

## Hesap ve Tavsiye
- Motor: 200 full steps/rev
- Microstep (şu an örnek): 16x → motor_steps_per_rev = 200 * 16 = 3200
- encoder_counts_per_step = 1600 / 3200 = 0.5 counts/microstep
  - Yorum: 0.5 counts/microstep demek, enkoder her tek mikroadımı tam olarak algılayamaz. Her 2 mikroadımda bir enkoder kenarı olur. Bu durumda kapalı çevrim pozisyon hassasiyeti sınırlı olabilir ve quantization (merdivenleme) görülebilir.

Tavsiye seçenekleri:
- Microstepping'i 8x yapın (motor_steps_per_rev = 1600) → encoder_counts_per_step = 1.0 (daha iyi)
- Veya daha yüksek CPR (ör. 1000 CPR) olan bir enkoder tercih edin
- Kontrol yazılımında hız rampası ve PID filtreleri kullanarak quantization etkisini azaltın

## İçerik (dosyalar)
- HEDS5540_Arduino.ino
- HEDS5540_Nano.ino
- HEDS5540_Nano_fast.ino
- measure_with_index.ino
- measure_manual_one_rev.ino
- closed_loop_simple_nano.ino
- closed_loop_nano_tb6600.ino
- README_HEDS5540.md (bu dosya)

## Hızlı Başlangıç
1. Donanım bağlayın:
   - Enkoder VCC -> 5V, GND -> GND, A -> D2, B -> D3 (harici 2.7 kΩ pull-up A/B → +5V)
   - TB6600 GND ile Arduino GND ortak olsun. TB6600 STEP -> D9, DIR -> D8, ENABLE -> D10 (opsiyonel)
2. `measure_with_index.ino` (Z varsa) veya `measure_manual_one_rev.ino` (Z yoksa) ile enkoder CPR doğrulaması yapın.
3. `closed_loop_nano_tb6600.ino` içindeki `encoder_counts_per_rev` değerinin 1600 (CPR 400 → 400*4) olduğundan emin olun.
4. TB6600 microstep DIP switch'ini istenen microstep (öncelikle 8x veya 16x) olarak ayarlayın. Eğer 0.5 counts/microstep görüyorsanız 8x tercih edin.
5. `closed_loop_nano_tb6600.ino`'yu yükleyin, Seri Monitör'ü 115200 ile açın.
6. Düşük hızlarda (küçük hedef step değişimleri) test edin ve PID parametrelerini yavaşça tune edin (ilk etapta Ki=0, Kp arttırarak deneyin).

## Notlar
- Enkoder beslemesi yaklaşık 57 mA çeker — 5V kaynağınızın kapasitesine dikkat edin.
- TB6600 step pulse minimum genişliğini sürücü dokümantasyonuna göre kontrol edin.
- Daha yüksek performans/kararlılık için LS7366R veya daha güçlü bir MCU (Teensy/STM32) önerilir.

---
Güncelleme tarihlemesi: 2025-11-13
