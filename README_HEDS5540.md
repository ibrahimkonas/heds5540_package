# HEDS-5540 Arduino Bağlantısı - Notlar

1. Datasheet'i kontrol edin:
   - Besleme voltajı, çıkış tipi (TTL / open-collector / differential), PPR/CPR bilgisi kesinlikle datasheet'te bulunur.

2. Kabellendirme:
   - Tek uçlu TTL: VCC -> 5V, GND -> GND, A -> D2, B -> D3, Z -> D4 (opsiyonel).
   - Eğer çıkış open-collector ise A/B/Z ile 5V arasına 4.7k–10k pull-up koyun.
   - Diferansiyel çıkışlı ise line receiver kullanın (SN75175 vb.)

3. Yazılım:
   - ISR içinde kısa, hızlı işler yapın (sayaç arttırma/azaltma).
   - Serial veya ağır işlemleri ISR dışında yapın.
   - countsPerRev değerini datasheet'ten alın: eğer cihaz "PPR" (cycles per rev) veriyorsa, quadrature ile countsPerRev = PPR * 4 olur.

4. Sorun giderme:
   - Hiç sinyal yok: VCC/GND kontrolü, pull-up, pinout doğru mu, kablo kopuk mu?
   - Sinyal çok gürültülü/dalgalı: ekranlı kablo, topraklama, RC veya Schmitt girişli alıcı kullan.
   - Yön ters: ISR'deki yön kontrolünü ters çevirin (if (a == b) position--; else position++;)
   - Çok yüksek hızda atlanan pulslar: Arduino kesmeleri yetmiyor olabilir -> donanım sayacı veya daha hızlı MCU önerilir.

5. Güvenlik:
   - Enkoderin izin verilen beslemesini aşmayın.
   - Besleme ve mantık seviyelerinin Arduino ile uyumlu olduğundan emin olun.