# Give a headless Raspberry Pi a new wifi network by editing its SD card.
#
# Put the Pi's microSD card in this laptop. Windows shows one drive called
# "bootfs" (ignore any "format this disk?" popup for the other partition, click
# Cancel). Then run, in PowerShell:
#
#   powershell -ExecutionPolicy Bypass -File tools/pi-wifi-rescue.ps1
#
# It asks for the wifi name and password, writes a one time script onto the
# card, and tells the Pi to run it on the next boot. That script saves the
# network into NetworkManager, removes itself, and reboots, so the second boot
# is a normal boot that joins the network. Same mechanism Raspberry Pi Imager
# uses for its first boot settings. Existing wifi networks are kept.

param(
    [string]$Ssid,
    [string]$Psk,
    [string]$CopyDir      # optional folder to deliver to /home/navi/<name> on the Pi
)

$ErrorActionPreference = "Stop"

$vol = Get-Volume | Where-Object { $_.FileSystemLabel -eq "bootfs" -and $_.DriveLetter } | Select-Object -First 1
if (-not $vol) {
    Write-Host "Could not find the SD card's 'bootfs' drive. Is the card in?" -ForegroundColor Red
    exit 1
}
$boot = "$($vol.DriveLetter):\"
$cmdline = Join-Path $boot "cmdline.txt"
if (-not (Test-Path $cmdline)) {
    Write-Host "$boot has no cmdline.txt, this does not look like a Pi boot partition." -ForegroundColor Red
    exit 1
}
Write-Host "Found the Pi boot partition at $boot"

$ssid = if ($Ssid) { $Ssid } else { Read-Host "Wifi name (exactly, e.g. Will's iPhone)" }
$psk = if ($Psk) { $Psk } else { Read-Host "Wifi password" }
if ($psk.Length -lt 8) {
    Write-Host "WPA passwords are at least 8 characters, check it." -ForegroundColor Red
    exit 1
}

# iPhones name their hotspot with a curly apostrophe. Typing a straight one on a
# PC is the most common reason this fails, so save both spellings.
$names = @($ssid)
if ($ssid -match "'") { $names += ($ssid -replace "'", [string][char]0x2019) }
if ($ssid -match [string][char]0x2019) { $names += ($ssid -replace [string][char]0x2019, "'") }

function Quote([string]$s) { "'" + ($s -replace "'", "'\''") + "'" }

$blocks = ""
$i = 0
foreach ($n in $names) {
    $i++
    $blocks += @"
UUID=`$(cat /proc/sys/kernel/random/uuid)
F=/etc/NetworkManager/system-connections/rescue-$i.nmconnection
cat > "`$F" <<EOF
[connection]
id=rescue-$i
uuid=`$UUID
type=wifi
autoconnect=true
autoconnect-priority=50

[wifi]
mode=infrastructure
ssid=`$(printf '%s' $(Quote $n))

[wifi-security]
key-mgmt=wpa-psk
psk=`$(printf '%s' $(Quote $psk))

[ipv4]
method=auto

[ipv6]
method=auto
EOF
chmod 600 "`$F"

"@
}

$copyBlock = ""
if ($CopyDir) {
    $leaf = Split-Path $CopyDir -Leaf
    $dest = Join-Path $boot $leaf
    if (Test-Path $dest) { Remove-Item $dest -Recurse -Force }
    Copy-Item $CopyDir $dest -Recurse
    Get-ChildItem $dest -Recurse -Directory -Filter __pycache__ | Remove-Item -Recurse -Force
    Write-Host "Copied $CopyDir to the card, it will land in /home/navi/$leaf"
    $copyBlock = @"
if [ -d /boot/firmware/$leaf ]; then
  rm -rf /home/navi/$leaf
  cp -r /boot/firmware/$leaf /home/navi/$leaf
  find /home/navi/$leaf -type f \( -name '*.sh' -o -name '*.py' \) -exec sed -i 's/\r`$//' {} +
  chmod +x /home/navi/$leaf/*.sh
  chown -R navi:navi /home/navi/$leaf
  rm -rf /boot/firmware/$leaf
fi
"@
}

$script = @"
#!/bin/bash
set +e
$blocks
$copyBlock
rfkill unblock wifi 2>/dev/null
sed -i 's| systemd.run.*||g' /boot/firmware/cmdline.txt /boot/cmdline.txt 2>/dev/null
rm -f /boot/firmware/navi-wifi.sh /boot/navi-wifi.sh
exit 0
"@

$utf8 = New-Object System.Text.UTF8Encoding($false)
[IO.File]::WriteAllText((Join-Path $boot "navi-wifi.sh"), ($script -replace "`r`n", "`n"), $utf8)

# cmdline.txt must stay a single line.
$line = ([IO.File]::ReadAllText($cmdline)).Trim()
Copy-Item $cmdline (Join-Path $boot "cmdline.txt.bak") -Force
$line = $line -replace ' systemd\.run.*', ''
$line += " systemd.run=/boot/firmware/navi-wifi.sh systemd.run_success_action=reboot systemd.unit=kernel-command-line.target"
[IO.File]::WriteAllText($cmdline, $line + "`n", $utf8)

Write-Host ""
Write-Host "Done. Saved: $($names -join '  /  ')" -ForegroundColor Green
Write-Host "1. Eject the card (right click the drive > Eject), put it back in the Pi."
Write-Host "2. Turn the hotspot on and keep that screen open on the phone."
Write-Host "3. Power the Pi. It boots, reboots once by itself, then joins. Give it 3 minutes."
Write-Host "4. Laptop on the same hotspot:  ssh navi@navi.local"
Write-Host ""
Write-Host "If the Pi never boots, put the card back in and rename cmdline.txt.bak to cmdline.txt."
