#!/bin/bash
export DISPLAY=:0

DIR="$HOME/Desktop/githubsal/"

if ! cd "$DIR"; then
    zenity --error --text="Hata: Klasör bulunamadı:\n$DIR"
    exit 1
fi

# İşlemi arka planda yap ve kullanıcıya pulsate bar göster
(
    # Pull işlemini yap ve hem çıktıları hem hataları değişkene al
    # 'rebase' kullanmak genellikle daha temiz bir geçmiş sağlar
    git pull origin main --rebase > /tmp/git_pull_log 2>&1
    echo "100"
) | zenity --progress --title="Senkronizasyon" --text="GitHub'dan veriler çekiliyor..." --pulsate --auto-close --no-cancel

# Sonucu kontrol et
CIKTI=$(cat /tmp/git_pull_log)

if [ $? -eq 0 ] && [[ ! $CIKTI == *"fatal"* ]] && [[ ! $CIKTI == *"error"* ]]; then
    zenity --info --text="✅ İşlem Başarılı!\n\nDetay:\n$CIKTI" --title="Git Pull Raporu" --width=500
else
    zenity --error --text="❌ Hata Oluştu!\n\nMuhtemelen internet hatası veya dosya çakışması var.\n\nDetay:\n$CIKTI" --title="Git Pull Hatası" --width=500
fi
