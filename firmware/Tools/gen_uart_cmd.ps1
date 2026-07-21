# SPDX-License-Identifier: MIT
# Copyright (c) 2024 The FOC Firmware Contributors
#
# gen_uart_cmd.ps1 -- UART command frame generator
# Run: powershell -File gen_uart_cmd.ps1
# Output: HEX strings, paste into serial terminal (HEX send, 115200 8N1)

function CRC16 {
    param([byte[]]$d)
    $c = [int]0xFFFF
    foreach ($b in $d) {
        $c = $c -bxor ([int]$b -shl 8)
        0..7 | % { $c = if ($c -band 0x8000) { (($c -shl 1) -bxor 0x1021) -band 0xFFFF } else { ($c -shl 1) -band 0xFFFF } }
    }
    return $c
}

function BuildFrame($mode, $deg, $kp, $kd) {
    if ($kp -eq $null) { $kp = 5.0 }
    if ($kd -eq $null) { $kd = 0.0 }
    $rad = $deg * [Math]::PI / 180.0
    $kpRaw = [int]($kp / 0.00015259 + 0.5)
    $kdRaw = [int]($kd / 0.00392157 + 0.5)

    $tb = [BitConverter]::GetBytes([single]$rad)
    $kpb = [BitConverter]::GetBytes([uint16]$kpRaw)

    $buf = [byte[]]::new(12)
    $buf[0] = 0xAA; $buf[1] = 0x08
    $buf[2] = $mode
    $buf[3] = $tb[0]; $buf[4] = $tb[1]; $buf[5] = $tb[2]; $buf[6] = $tb[3]
    $buf[7] = $kpb[0]; $buf[8] = $kpb[1]
    $buf[9] = [byte]($kdRaw -band 0xFF)

    $crc = CRC16 $buf[0..9]
    $buf[10] = [byte]($crc -band 0xFF)
    $buf[11] = [byte](($crc -shr 8) -band 0xFF)
    return $buf
}

# ---- 常用命令 ----
Write-Host "===== 串口命令 (HEX) ====="
Write-Host "115200 8N1, 选 HEX 发送`n"

$names = @("待机", "0°  ", "10° ", "45° ", "90° ", "校准")
$modes = @(0, 3, 3, 3, 3, 4)
$degs  = @(0, 0, 10, 45, 90, 0)

for ($i = 0; $i -lt $names.Length; $i++) {
    $f = BuildFrame $modes[$i] $degs[$i] $null $null
    $hex = ($f | ForEach-Object { $_.ToString('X2') }) -join ' '
    Write-Host ("{0,-6}: {1}" -f $names[$i], $hex)
}
