import os
import subprocess
import sys
import shutil
import re
# --- RENKLENDİRME ---
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

# --- YARDIMCI FONKSİYONLAR ---
def get_real_user():
    return os.environ.get('SUDO_USER') or os.getlogin()

def get_workspace_paths():
    real_user = get_real_user()
    home_dir = os.path.expanduser(f"~{real_user}")
    ws_path = os.path.join(home_dir, "Desktop", "my_robot_ws")
    src_path = os.path.join(ws_path, "src")
    return ws_path, src_path, real_user

def run_command(command, shell=False, as_user=None):
    try:
        if as_user:
            cmd = f"sudo -u {as_user} bash -c '{command}'" if shell else ["sudo", "-u", as_user] + command
            subprocess.check_call(cmd, shell=shell)
        else:
            subprocess.check_call(command, shell=shell)
    except subprocess.CalledProcessError as e:
        print(f"{Colors.FAIL}[HATA] Komut başarısız oldu: {e}{Colors.ENDC}")

def install_apt(packages):
    if isinstance(packages, str): packages = [packages]
    to_install = []
    for pkg in packages:
        try:
            subprocess.check_output(f"dpkg -s {pkg}", shell=True, stderr=subprocess.DEVNULL)
        except:
            to_install.append(pkg)
    if to_install:
        print(f"{Colors.WARNING}[APT] Kuruluyor: {', '.join(to_install)}{Colors.ENDC}")
        run_command(["apt-get", "install", "-y"] + to_install)

# --- ADIM FONKSİYONLARI ---

def step_1_repos():
    print(f"\n{Colors.HEADER}--- 1. SİSTEM REPOLARI ---{Colors.ENDC}")
    run_command("apt-get update", shell=True)
    install_apt(["curl", "wget", "lsb-release", "gnupg", "software-properties-common", "tree"])
    if not os.path.exists("/etc/apt/sources.list.d/ros2.list"):
        run_command("curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg", shell=True)
        run_command('echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null', shell=True)
    run_command("apt-get update", shell=True)

def step_2_packages():

    essential_tools = ["ros-humble-desktop", "python3-colcon-common-extensions", "python3-rosdep", "python3-pip"]
    install_apt(essential_tools)
    sim_packages = ["gz-harmonic", "ros-humble-ros-gz", "ros-humble-xacro", "ros-humble-robot-state-publisher"]
    install_apt(sim_packages)

def step_3_vrx_download():
    print(f"\n{Colors.HEADER}--- 3. VRX KAYNAK KODU ---{Colors.ENDC}")
    ws_path, src_path, real_user = get_workspace_paths()
    vrx_target = os.path.join(src_path, "vrx")
    os.makedirs(src_path, exist_ok=True)
    shutil.chown(ws_path, real_user, real_user)
    if not os.path.exists(vrx_target):
        run_command(f"git clone https://github.com/osrf/vrx.git {vrx_target}", shell=True, as_user=real_user)
    else:
        print(f"{Colors.OKGREEN}[TAMAM] VRX zaten mevcut.{Colors.ENDC}")

def step_4_vision():
    run_command([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])
    
    packages = ["ultralytics==8.4.6", "depthai", "opencv-python"]
    cmd = [sys.executable, "-m", "pip", "install"] + packages + ["--ignore-installed", "--break-system-packages"]
    
    run_command(cmd)
    
def step_5_build():
    print(f"\n{Colors.HEADER}--- 5. WORKSPACE DERLEME (HAFİF VE GÜVENLİ) ---{Colors.ENDC}")
    ws_path, _, real_user = get_workspace_paths()
    
    # Rasyonel Kısıtlamalar:
    # 1. GZ_VERSION=harmonic -> Köprünün doğru dilde derlenmesi için
    # 2. MAKEFLAGS="-j2" -> C++ derleyicisinin sadece 2 CPU çekirdeği kullanması için
    # 3. --parallel-workers 2 -> Colcon'un aynı anda sadece 2 paket derlemesi için
    # 4. -DBUILD_TESTING=OFF -> Gereksiz test dosyalarını atlayıp zaman/RAM kazanmak için
    
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"cd {ws_path} && "
        f"export GZ_VERSION=harmonic && "
        f"export MAKEFLAGS=\"-j2\" && "
        f"colcon build --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF"
    )
    
    run_command(cmd, shell=True, as_user=real_user)
def step_6_rename():
    print(f"\n{Colors.HEADER}--- 6. İSİM VE YOL DÜZENLEME ---{Colors.ENDC}")
    _, src_path, real_user = get_workspace_paths()
    target = os.path.join(src_path, "my_robot_description")
    
    if not os.path.exists(target):
        print(f"{Colors.FAIL}Hata: {target} bulunamadı!{Colors.ENDC}")
        return

    # Kullanıcıdan isimleri alıyoruz
    old_n = input(f"{Colors.OKBLUE}Dosyalarda kayıtlı olan eski kullanıcı adı (varsayılan: gauss): {Colors.ENDC}") or "gauss"
    new_n = input(f"{Colors.OKBLUE}Senin bilgisayarındaki kullanıcı adın (varsayılan: {real_user}): {Colors.ENDC}") or real_user

    # Regex Güvencesi: Sadece tam yol eşleşmelerini (örn: /home/gauss/) yakalar.
    # Bu sayede dosya içindeki 'thruster' veya 'joint' gibi kelimeler ASLA bozulmaz.
    home_pattern = re.compile(rf'/home/{old_n}/')
    new_home = f'/home/{new_n}/'
    
    count = 0
    for r, d, files in os.walk(target):
        for f in files:
            if f.endswith(('.sdf', '.launch.py', '.xml', '.yaml', '.config', '.urdf', '.xacro')):
                path = os.path.join(r, f)
                
                with open(path, 'r', encoding='utf-8') as file:
                    content = file.read()
                
                # Değişiklik kontrolü
                if home_pattern.search(content) or "gaussas" in content:
                    # Sadece /home/eski_isim/ formatını senin yeni isminle değiştirir
                    new_content = home_pattern.sub(new_home, content)
                    
                    # Varsa özel mesh isimlendirmelerini düzeltir
                    new_content = new_content.replace("gaussas", "denemeas")
                    
                    with open(path, 'w', encoding='utf-8') as file:
                        file.write(new_content)
                    count += 1

    print(f"{Colors.OKGREEN}[TAMAM] {count} dosyada yollar '{new_n}' olarak güncellendi.{Colors.ENDC}")

def step_7_bashrc_fix():
    print(f"\n{Colors.HEADER}--- 7. KALICI BASHRC AYARLARI ---{Colors.ENDC}")
    ws_path, _, real_user = get_workspace_paths()
    bashrc = os.path.expanduser(f"~{real_user}/.bashrc")
    
    # Plugin yolunu find ile dinamik bulalım (garanti olsun)
    plugin_subpath = "install/vrx_gz/lib/"
    
    lines = [
        f"\n# --- ROBOT WS AYARLARI ---",
        f"source /opt/ros/humble/setup.bash",
        f"source {ws_path}/install/setup.bash",
        f"export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:{ws_path}/{plugin_subpath}",
        f"export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:{ws_path}/src/my_robot_description/models",
        f"export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:{ws_path}/src/my_robot_description/worlds"
    ]
    
    with open(bashrc, "r") as f: content = f.read()
    with open(bashrc, "a") as f:
        for l in lines:
            if l.strip() not in content:
                f.write(l + "\n")
    print(f"{Colors.OKGREEN}[TAMAM] Yollar .bashrc dosyasına eklendi.{Colors.ENDC}")

def step_8_run_simulation():
    print(f"\n{Colors.HEADER}--- 8. SİMÜLASYON BAŞLATILIYOR ---{Colors.ENDC}")
    ws_path, _, real_user = get_workspace_paths()
    world_file = "final_boat_hydro.sdf"
    world_dir = os.path.join(ws_path, "src/my_robot_description/worlds")
    
    env = os.environ.copy()
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')}:{ws_path}/install/vrx_gz/lib/"
    env["GZ_SIM_RESOURCE_PATH"] = f"{env.get('GZ_SIM_RESOURCE_PATH','')}:{ws_path}/src/my_robot_description/models:{world_dir}"
    
    print(f"{Colors.OKBLUE}Dünya yükleniyor: {world_file}{Colors.ENDC}")
    try:
        # Simülasyonu normal kullanıcı olarak başlat (GUI için önemli)
        cmd = f"gz sim -r {world_file}"
        subprocess.run(f"sudo -u {real_user} bash -c 'source /opt/ros/humble/setup.bash && source {ws_path}/install/setup.bash && {cmd}'", 
                       shell=True, cwd=world_dir, env=env)
    except Exception as e:
        print(f"Hata: {e}")
        
def clean_gazebo_basic():
    """Donan veya çakışan temel Gazebo süreçlerini öldürür."""
    print(f"\n{Colors.WARNING}--- TEMEL GAZEBO TEMİZLİĞİ ---{Colors.ENDC}")
    # Süreç bulunamazsa hata dönmemesi için || true kullanıyoruz
    commands = [
        "pkill -9 -f ign || true",
        "pkill -9 -f gazebo || true",
        "pkill -9 -f ruby || true"
    ]
    for cmd in commands:
        run_command(cmd, shell=True)
    print(f"{Colors.OKGREEN}Süreçler sonlandırıldı.{Colors.ENDC}")

def clean_gazebo_deep():
    """Süreçlerle birlikte paylaşımlı hafıza ve geçici dosyaları da temizler."""
    print(f"\n{Colors.FAIL}--- KÖKTEN GAZEBO TEMİZLİĞİ (DEEP CLEAN) ---{Colors.ENDC}")
    
    # 1. Tüm ilgili süreçleri tek seferde uçur
    run_command("pkill -9 -f 'ign|gazebo|ruby|gz' || true", shell=True)
    
    # 2. Gazebo'nun ulaşım katmanı (transport layer) artıklarını temizle
    # /dev/shm altındaki dosyalar temizlenmezse yeni simülasyon eski portlara bağlanmaya çalışabilir
    cleanup_files = [
        "/dev/shm/ign-*",
        "/dev/shm/gz-*",
        "/tmp/.ignition",
        "/tmp/.gz"
    ]
    
    # Dosyaları sil
    run_command(f"rm -rf {' '.join(cleanup_files)} || true", shell=True)
    
    print(f"{Colors.OKGREEN}Sistem tamamen temizlendi ve yeni simülasyona hazır!{Colors.ENDC}")

def full_auto_setup():
    print(f"\n{Colors.WARNING}!!! FULL OTOMATİK KURULUM BAŞLIYOR !!!{Colors.ENDC}")
    step_1_repos()
    step_2_packages()
    step_3_vrx_download()
    step_4_vision()
    step_5_build()
    step_6_rename(auto=True)
    step_7_bashrc_fix()
    print(f"\n{Colors.OKGREEN}✅ TÜM KURULUM BAŞARIYLA TAMAMLANDI!{Colors.ENDC}")
    print("Simülasyonu başlatmak için 8. adımı kullanabilirsin.")

# --- ANA MENÜ ---
def main():
    if os.geteuid() != 0:
        print(f"{Colors.FAIL}Bu scripti SUDO ile çalıştırmalısın!{Colors.ENDC}")
        sys.exit(1)

    while True:
        print(f"\n{Colors.HEADER}=== GAUSS ROBOTICS SETUP TOOL ==={Colors.ENDC}")
        print(f"{Colors.OKGREEN}A. FULL OTOMATİK KURULUM (Adım 1-7){Colors.ENDC}")
        print("-----------------------------------")
        print("1. Repoları Ekle")
        print("2. Paketleri Kur")
        print("3. VRX İndir")
        print("4. Vision Stack Kur")
        print("5. Workspace Derle (Build)")
        print("6. İsim/Yol Düzenle")
        print("7. Bashrc Kalıcı Yap")
        print("8. Simülasyonu Başlat")    
        print("9. Temel Temizlik (pkill)")
        print("10. Sağlam Temizlik (Deep Wipe)")  
        print("Q. Çıkış")
        
        choice = input(f"\n{Colors.OKBLUE}Seçiminiz: {Colors.ENDC}").strip().lower()

        if choice == 'a': full_auto_setup()
        elif choice == '1': step_1_repos()
        elif choice == '2': step_2_packages()
        elif choice == '3': step_3_vrx_download()
        elif choice == '4': step_4_vision()
        elif choice == '5': step_5_build()
        elif choice == '6': step_6_rename()
        elif choice == '7': step_7_bashrc_fix()
        elif choice == '8': step_8_run_simulation()
        elif choice == '9': clean_gazebo_basic()
        elif choice == '10': clean_gazebo_deep()
        elif choice == 'q': break
        else: print(f"{Colors.FAIL}Geçersiz seçenek!{Colors.ENDC}")

if __name__ == "__main__":
    main()
