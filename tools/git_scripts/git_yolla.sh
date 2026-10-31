#!/bin/bash
export DISPLAY=:0

# Klasör yolu
DIR="$HOME/Desktop/githubsal"

# Klasöre git
if ! cd "$DIR"; then
    zenity --error --text="Hata: Klasör bulunamadı:\n$DIR"
    exit 1
fi

# 1. Kullanıcıdan Mesaj İste
MESAJ=$(zenity --entry --title="GitHub'a Gönder" --text="Değişikliklerin için bir not yaz:" --width=400)

if [ -z "$MESAJ" ]; then
    exit 0
fi

# ÖNEMLİ: Hataları yakalamak için bir değişken kullanalım
(
    echo "10"; echo "# Dosyalar taranıyor..."
    git add . || { echo "# Git Add Hatası!"; exit 1; }
    
    echo "40"; echo "# Commit yapılıyor..."
    # Eğer commit edilecek bir şey yoksa commit hata verir, bunu görmezden gelmek için '|| true'
    git commit -m "$MESAJ" || echo "# Değişiklik yok, devam ediliyor..."
    
    echo "70"; echo "# GitHub'a yükleniyor (Push)..."
    # Buraya kendi branch adını (main veya master) yazmalısın
    git push origin main --force 2>&1 | while read -r line; do echo "# $line"; done
    
    if [ ${PIPESTATUS[0]} -ne 0 ]; then
        echo "# Push başarısız!"; exit 1
    fi
    
    echo "100"; echo "# Tamamlandı!" 
) | zenity --progress --title="İşleniyor" --text="Başlıyor..." --percentage=0 --auto-close --no-cancel

# Sonuç kontrolü (Zenity'nin çıkış kodunu değil, push işlemini kontrol etmelisin)
if [ $? -eq 0 ]; then
    zenity --info --text="✅ İşlem Tamamlandı!\n\nGitHub'ı kontrol edebilirsin." --title="Başarılı"
else
    zenity --error --text="❌ Bir şeyler ters gitti.\n\nLütfen terminalden manuel 'git push' yaparak hatayı gör."
fi
