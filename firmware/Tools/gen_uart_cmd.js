// SPDX-License-Identifier: MIT
// Copyright (c) 2024 The FOC Firmware Contributors
//
// gen_uart_cmd.js -- UART command frame generator
// Usage: cscript //nologo gen_uart_cmd.js
// Output: HEX strings, paste into serial terminal (HEX send, 115200 8N1)

function crc16(data) {
    var crc = 0xFFFF;
    for (var i = 0; i < data.length; i++) {
        crc ^= data[i] << 8;
        for (var j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
            else
                crc = (crc << 1) & 0xFFFF;
        }
    }
    return crc;
}

function floatToBytes(f) {
    // IEEE 754 single precision, little-endian
    var buf = new Array(4);
    var sign = (f < 0) ? 1 : 0;
    if (sign) f = -f;

    if (f === 0) return [0, 0, 0, 0];

    var e = 0;
    var m = f;
    while (m >= 2) { m /= 2; e++; }
    while (m < 1)  { m *= 2; e--; }

    var exp = e + 127;  // biased exponent
    if (exp >= 255) {  // infinity
        buf[0] = 0; buf[1] = 0; buf[2] = 0x80; buf[3] = 0x7F;
        if (sign) buf[3] = 0xFF;
        return buf;
    }
    if (exp <= 0) { m /= 2; exp = 0; }  // subnormal

    var mant = Math.round((m - 1) * 0x800000);
    if (mant >= 0x800000) { mant = 0; exp++; }

    var bits = (sign << 31) | (exp << 23) | mant;
    buf[0] = bits & 0xFF;
    buf[1] = (bits >> 8) & 0xFF;
    buf[2] = (bits >> 16) & 0xFF;
    buf[3] = (bits >> 24) & 0xFF;
    return buf;
}

function uint16bytes(v) {
    return [v & 0xFF, (v >> 8) & 0xFF];
}

function build_cmd(mode, target_deg, kp, kd) {
    if (kp === undefined) kp = 5.0;
    if (kd === undefined) kd = 0.0;

    var target_rad = target_deg * 3.1415926535 / 180.0;
    var kp_raw = Math.round(kp / 0.00015259);
    var kd_raw = Math.round(kd / 0.00392157);

    var tb = floatToBytes(target_rad);
    var kb = uint16bytes(kp_raw);

    var frame = [
        0xAA, 0x08,
        mode,
        tb[0], tb[1], tb[2], tb[3],
        kb[0], kb[1],
        kd_raw & 0xFF
    ];

    var crc = crc16(frame);
    var full = frame.concat([crc & 0xFF, (crc >> 8) & 0xFF]);

    var hex = "";
    for (var i = 0; i < full.length; i++) {
        var s = full[i].toString(16).toUpperCase();
        if (s.length < 2) s = "0" + s;
        hex += s;
        if (i < full.length - 1) hex += " ";
    }
    return hex;
}

var cmds = [
    ["STANDBY", 0, 0],
    ["0deg   ", 3, 0],
    ["10deg  ", 3, 10],
    ["45deg  ", 3, 45],
    ["90deg  ", 3, 90],
    ["CALIB  ", 4, 0]
];

WScript.Echo("===== UART Commands =====");
WScript.Echo("115200 8N1, send as HEX");
WScript.Echo("");
for (var i = 0; i < cmds.length; i++) {
    WScript.Echo(cmds[i][0] + ": " + build_cmd(cmds[i][1], cmds[i][2]));
}
