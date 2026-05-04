# CNC Digital Twin — Çalıştırma Rehberi

## Proje dosyaları

```
cnc_digital_twin/
│
├── cnc_twin_full.html          ← Three.js + Analytics (tek dosya, kurulum yok)
├── cnc_twin_analytics.html     ← Sadece Plotly analytics dashboard
├── cnc_digital_twin.html       ← Sadece Three.js 3D sahne (ilk versiyon)
│
└── streamlit/
    ├── app.py                  ← Streamlit uygulaması
    └── requirements.txt        ← Python bağımlılıkları
```

---

## 1. HTML dosyaları (kurulum yok, sıfır bağımlılık)

### cnc_twin_full.html — Ana uygulama ⭐
Three.js 3D sahne + Plotly analytics + zaman şeridi, hepsi bir arada.

**Çalıştırmak için:**
```
Dosyayı indir → Chrome veya Firefox'ta aç → ▶ Run butonuna bas
```

**Özellikler:**
- 3D sahne: 12 CNC makine, 3 hat, 2 montaj, final assembly
- 📊 Analytics butonu: Plotly grafiklere geçiş
- Alt şerit: Gün/saat/vardiya, OEE renkli bar, batch countdown
- Sidebar: KPIs, Machines (per-makine slider), Flow, Params
- Drag: orbit kamera, scroll: zoom, ⟳ View: preset açılar

### cnc_twin_analytics.html — Sadece analitik
```
Dosyayı indir → Chrome'da aç → ▶ Run → sekmeler arası geç
```

### cnc_digital_twin.html — Sadece 3D (ilk PoC)
```
Dosyayı indir → Chrome'da aç → ▶ Run
```

---

## 2. Streamlit uygulaması

### Gereksinimler
- Python 3.9+
- pip

### Kurulum (ilk kez)
```bash
# 1. Klasör oluştur
mkdir cnc_streamlit && cd cnc_streamlit

# 2. Dosyaları koy: app.py ve requirements.txt

# 3. Sanal ortam oluştur (opsiyonel ama önerilir)
python -m venv venv
source venv/bin/activate        # Mac/Linux
venv\Scripts\activate           # Windows

# 4. Bağımlılıkları yükle
pip install -r requirements.txt
```

### Çalıştırma
```bash
streamlit run app.py
```
Tarayıcı otomatik açılır: http://localhost:8501

### requirements.txt içeriği
```
streamlit>=1.32.0
plotly>=5.18.0
pandas>=2.0.0
numpy>=1.24.0
```

---

## 3. Parametreler — ne değiştirir ne

### Per-makine (Machines sekmesi / sidebar)
| Parametre | Etki |
|-----------|------|
| Cycle time (min) | Bir parçayı işleme süresi. Yüksek → o makine darboğaz olur |
| Breakdown % | Her döngüde arıza çıkma ihtimali. Yüksek → OEE düşer |
| Repair time (min) | Arıza sonrası bekleme. Uzun → availability düşer |
| Energy (kW) | Toplam enerji tüketimine katkı |

### Sistem geneli (Params sekmesi)
| Parametre | Etki |
|-----------|------|
| Line A/B/C merge | Montaj için gereken minimum parça. Düşük → hızlı ama sık setup |
| Final merge | Final assembly için gereken sub-assembly sayısı |
| Transfer time | İstasyonlar arası taşıma süresi |
| Batch (days) | Toplu temizleme periyodu |
| Eff. sigma | Makine verimlilik dağılım genişliği. Yüksek → daha gerçekçi ama daha kaotik |
| Primary/Secondary recovery | OEE kalite bileşeni. Düşük → OEE düşer |

---

## 4. MQTT entegrasyonu (gelecek adım)

HTML dosyasında şu kısım gerçek veriyle değişecek:
```javascript
// Şu an: simülasyon verisi
if(running && des.pq.size) {
    const evts = des.step(800, tgt);
    updateKPIs(des.getKPIs());
}

// MQTT gelince: gerçek veri
mqttClient.on('message', (topic, payload) => {
    const data = JSON.parse(payload);
    // aynı updateKPIs() fonksiyonu çağrılır
    // Three.js sahne değişmez
    updateKPIs(data);
});
```

Streamlit tarafında:
```python
# paho-mqtt ile broker bağlantısı
import paho.mqtt.client as mqtt

def on_message(client, userdata, msg):
    data = json.loads(msg.payload)
    st.session_state['live_kpi'] = data
```

---

## 5. Sık sorulan sorular

**Q: Dosyayı açınca boş geliyor?**
A: Chrome veya Firefox kullan. Safari'de WebGL kısıtlaması olabilir.

**Q: Simülasyon çok yavaş çalışıyor?**
A: Speed slider'ı 200-400x'e çek. Toolbar'da sol üstte.

**Q: OEE neden düşük (%50 civarı)?**
A: Params sekmesinden Primary recovery → 99%, Secondary recovery → 99% yap.
Veya Machines sekmesinden Breakdown % değerlerini düşür.

**Q: Darboğaz nasıl tespit edilir?**
A: Kırmızıya dönen makine = kritik darboğaz. KPI sekmesindeki bar chart sıralı gösterir.

**Q: Batch cleanup nedir?**
A: Her N günde bir kuyruklar sıfırlanır. Zaman şeridinde sarı çizgi bir sonraki cleanup'ı gösterir.
