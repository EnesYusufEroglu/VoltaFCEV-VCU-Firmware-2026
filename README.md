# VoltaFCEV-VCU-Firmware-2026
Bu proje, TEKNOFEST Elektrikli Araç Yarışması'nda mücadele eden **VoltaFCEV** takımının elektrikli yarış aracının Araç Kontrol Ünitesi (VCU) için geliştirilmiş gömülü yazılım (firmware) projesidir.

Sistem, araç içi sensör verilerinin toplanması, motor sürücü entegrasyonu, Batarya Yönetim Sistemi (BYS) takibi ve verilerin yer istasyonuna gerçek zamanlı kablosuz aktarılmasından sorumludur.

## 🛠 Donanım ve Yazılım Mimarisi
- **Mikrodenetleyici:** STM32F407VGT6 (ARM Cortex-M4)
- **Geliştirme Ortamı:** Gömülü C (Embedded C) / HAL & Register Seviyesi
- **IDE:** STM32CubeIDE & STM32CubeMX

## 📡 Kullanılan Haberleşme Protokolleri & Çevre Birimleri
Proje mimarisi, endüstriyel standartlarda yüksek güvenilirlikli ve düşük gecikmeli veri akışı sağlamak üzere kesme (interrupt) tabanlı tasarlanmıştır:

* **CAN Bus (Controller Area Network):** Araç içi sistemler (Motor Sürücü, BYS) arasında 250 kbps / 500 kbps bant genişliğinde gerçek zamanlı ve güvenli veri paketlemesi.
* **UART (Universal Asynchronous Receiver-Transmitter):** Kablosuz LoRa modülleri (E32-433T30D) ile yer istasyonu arasında asenkron veri transferi.
* **SPI / I2C:** Çevre sensör verilerinin (sıcaklık, ivme, akım/gerilim sensörleri) düşük gürültülü hatlar üzerinden okunması.

## 📊 Telemetri Veri Protokolü (Data Frame Map)
Yer istasyonuna LoRa üzerinden gönderilen veriler, kablosuz haberleşme gürültüsünden arındırılmak ve veri bütünlüğünü korumak adına özel bir veri çerçevesi (Data Frame) ile paketlenir:

| Byte İndeksi | Tanım | Veri Tipi | Fonksiyon / Detay |
| :--- | :--- | :--- | :--- |
| `0x00` | Header (Başlık) | `uint8_t` | Paket başlangıç kontrol baytı (`0xAA`) |
| `0x01 - 0x02` | Motor RPM | `uint16_t` | Aracın anlık motor hızı |
| `0x03 - 0x04` | Batarya Voltajı | `uint16_t` | BMS üzerinden okunan toplam gerilim (V) |
| `0x05` | Sıcaklık Verisi | `int8_t` | Motor ve sürücü sıcaklık logları (°C) |
| `0x06` | Araç Durum Baytı | `uint8_t` | Hata kodları, sensör durumları ve bayraklar (Flags) |
| `0x07 - 0x08` | CRC/Checksum | `uint16_t` | Veri bütünlüğü doğrulama (Hata kontrolü) |

## 🚀 Öne Çıkan Gömülü Yazılım Özellikleri
- **Kesme (Interrupt) Tabanlı Mimari:** Seri port ve CAN-Bus haberleşmesi, işlemciyi bloke etmemek adına `HAL_UART_Receive_IT` ve CAN Rx FIFO kesmeleri ile asenkron olarak yönetilir.
- **Veri Bütünlüğü Güvencesi:** Paket kayıplarını ve hat üstündeki bozulmaları önlemek amacıyla donanımsal veya yazılımsal CRC/Checksum algoritmaları entegre edilmiştir.
- **Modüler Sürücü Tasarımı:** LoRa modülleri ve HMI (DWIN vb.) ekran arabirimleri için harici donanım bağımlılığını minimize eden, taşınabilir C kütüphaneleri (sürücüler) sıfırdan kurgulanmıştır.

## 📂 Proje Yapısı
- `/Core/Src`: Ana döngü (`main.c`), kesme rutinleri (`stm32f4xx_it.c`) ve takıma özel donanım sürücüleri (`Lora.c`, `Canbus.c`).
- `/Core/Inc`: Çevre birimlerine ait konfigürasyon ve fonksiyon tanımlamalarını içeren başlık (`.h`) dosyaları.
- `/Drivers`: STM32 HAL ve CMSIS donanım soyutlama kütüphaneleri.
- `*.ioc`: Donanım pin eşleşmeleri ve saat ağacını (Clock Tree) içeren CubeMX yapılandırma dosyası.
