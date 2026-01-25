// server-v5.js
// UDP <-> WebSocket bridge for AES67 mixer status/control + channel names
// - CTRL v0x02: gain_u10 (u16, 0..1023), pan_s8 (s8, -127..127), mute_mask, solo_mask
// - STAT v0x05: removed ptp_offset_ns / ptp_rate_ppb; added samples_read/samples_written
// - Writes a CSV logfile on status reception

'use strict';

const dgram = require('dgram');
const WebSocket = require('ws');
const fs = require('fs');

const STATUS_PORT   = 7800;               // FPGA -> PC (status)
const CONTROL_PORT  = 7880;               // PC -> FPGA (control)
const NAME_PORT     = 7880;               // PC -> FPGA (channel names)
const FPGA_IP       = '192.168.1.128';    // <-- anpassen
const WS_PORT       = 8080;

// UDP sockets
const udpStatusSock  = dgram.createSocket('udp4');
const udpControlSock = dgram.createSocket('udp4');
const udpNameSock    = dgram.createSocket('udp4');

// WebSocket server
const wss = new WebSocket.Server({ port: WS_PORT });

console.log(`WebSocket server listening on ws://0.0.0.0:${WS_PORT}`);
console.log(`Listening for status UDP on 0.0.0.0:${STATUS_PORT}`);
console.log(`Sending control UDP to ${FPGA_IP}:${CONTROL_PORT}`);
console.log(`Sending name   UDP to ${FPGA_IP}:${NAME_PORT}`);

// Connected clients
const clients = new Set();

// Global state
let gains = new Array(16).fill(1023);   // 10-bit fader pos 0..1023
let pans  = new Array(16).fill(0);      // s8 -127..127 (0=center)
let mutes = new Array(16).fill(false);
let solos = new Array(16).fill(false);
let soloMask = 0;                       // u16 one-hot or 0
let names = new Array(16).fill('');

function maskToBoolArray(mask16) {
    const out = new Array(16);
    const m = (mask16 >>> 0) & 0xFFFF;
    for (let i=0; i<16; i++) out[i] = (((m >>> i) & 1) === 1);
    return out;
}

// ---------- Helpers ----------
function clamp(x, lo, hi) { return Math.max(lo, Math.min(hi, x)); }

function readAsciiZ(buf, offset, len) {
    const slice = buf.slice(offset, offset + len);
    let s = slice.toString('ascii');
    const nul = s.indexOf('\0');
    if (nul >= 0) s = s.slice(0, nul);
    return s.trimEnd();
}

function broadcast(obj) {
    const s = JSON.stringify(obj);
    for (const ws of clients) {
        if (ws.readyState === WebSocket.OPEN) ws.send(s);
    }
}

function nowIso() {
    // millisecond precision, stable for CSV
    return new Date().toISOString();
}

// ---------- CSV logging ----------
const LOG_FILE = 'status_log.csv';
const CSV_HEADER = [
    'recv_time_iso',
    'ptp_sec',
    'ptp_ns',
    'samples_read',
    'samples_written',
    'stat_late',
    'stat_early',
    'stat_overwrite',
    'stat_underrun',
    'buffer_fill',
    'spdif_samples',
    'xfade_main_cnt',
    'xfade_extra_cnt',
    'xfade_dup_cnt',
    'xfade_zero_cnt',
    'gains',
    'pans'
].join(',') + '\n';

function ensureCsvHeader() {
    try {
        const st = fs.statSync(LOG_FILE);
        if (st.size === 0) fs.appendFileSync(LOG_FILE, CSV_HEADER);
    } catch (e) {
        // file doesn't exist
        fs.writeFileSync(LOG_FILE, CSV_HEADER);
    }
}
ensureCsvHeader();

function appendCsvRow(fields) {
    // fields already sanitized numbers/strings
    const line = fields.map(v => String(v)).join(',') + '\n';
    fs.appendFile(LOG_FILE, line, err => {
        if (err) console.error('CSV append error:', err);
    });
}

// ---------- WebSocket ----------
wss.on('connection', ws => {
    console.log('WebSocket client connected');
    clients.add(ws);

    ws.on('close', () => {
        console.log('WebSocket client disconnected');
        clients.delete(ws);
    });

    ws.on('message', msg => {
        try {
            const obj = JSON.parse(msg.toString());
            handleWsMessage(obj);
        } catch (e) {
            console.error('Invalid WS message', e);
        }
    });

    ws.send(JSON.stringify({
        type: 'init',
        gains,
        pans,
        mutes,
        solos,
        soloMask,
        names
    }));
});

function handleWsMessage(msg) {
    if (msg.type === 'setAll' && Array.isArray(msg.gains) && Array.isArray(msg.pans)) {
        gains = msg.gains.slice(0, 16).map(v => clamp(v|0, 0, 1023));
        pans  = msg.pans.slice(0, 16).map(v => clamp(v|0, -127, 127));
        if (Array.isArray(msg.mutes)) mutes = msg.mutes.slice(0,16).map(v => !!v);
        if (typeof msg.soloMask === 'number') soloMask = msg.soloMask & 0xFFFF;
        sendControlPacket(gains, pans, mutes, soloMask);
    } else if (msg.type === 'setChannel') {
        const ch = msg.channel | 0;
        if (ch < 0 || ch >= 16) return;
        if (typeof msg.gain === 'number') gains[ch] = clamp(msg.gain|0, 0, 1023);
        if (typeof msg.pan  === 'number') pans[ch]  = clamp(msg.pan|0, -127, 127);
        if (typeof msg.mute === 'boolean') mutes[ch] = msg.mute;

        // soloMask is managed client-side (one-hot or 0)
        if (typeof msg.soloMask === 'number') soloMask = msg.soloMask & 0xFFFF;

        sendControlPacket(gains, pans, mutes, soloMask);
    } else if (msg.type === 'setName') {
        const ch = msg.channel | 0;
        if (ch < 0 || ch >= 16) return;
        const name = String(msg.name || '');
        names[ch] = name;
        sendNamePacket(ch, name);
    }
}

// ---- CONTROL (Gain/Pan/Mute/Solo) ----
function sendControlPacket(gainArr, panArr, muteArr, soloMaskIn) {
    // CTRL v0x02 (60 Bytes)
    const buf = Buffer.alloc(60);
    buf.write('CTRL', 0, 4, 'ascii');
    buf[4] = 0x02;
    buf[5] = 0x00; buf[6] = 0x00; buf[7] = 0x00;

    for (let i = 0; i < 16; i++) {
        const g = clamp(Number(gainArr[i] || 0) | 0, 0, 1023);
        buf.writeUInt16BE(g, 8 + i*2);
    }

    for (let i = 0; i < 16; i++) {
        const p = clamp(Number(panArr[i] || 0) | 0, -127, 127);
        buf.writeInt8(p, 40 + i);
    }

    let muteMask = 0;
    if (Array.isArray(muteArr)) {
        for (let i=0; i<16; i++) if (muteArr[i]) muteMask |= (1<<i);
    }
    buf.writeUInt16BE(muteMask & 0xFFFF, 56);

    buf.writeUInt16BE((soloMaskIn|0) & 0xFFFF, 58);

    udpControlSock.send(buf, CONTROL_PORT, FPGA_IP, err => {
        if (err) console.error('UDP control send error', err);
    });
}

// ---- NAME (Channel Names) ----
function sendNamePacket(ch, name) {
    const buf = Buffer.alloc(24);
    buf.write('NAME', 0, 4, 'ascii');
    buf[4] = 0x01;
    buf[5] = ch & 0xFF;
    buf[6] = 0x00;
    buf[7] = 0x00;

    const nameBuf = Buffer.from(name, 'utf8');
    const len = Math.min(16, nameBuf.length);
    nameBuf.copy(buf, 8, 0, len);

    udpNameSock.send(buf, NAME_PORT, FPGA_IP, err => {
        if (err) console.error('UDP name send error', err);
    });
}

// ---- STATUS RX (FPGA -> Web) ----
udpStatusSock.on('message', (msg, rinfo) => {
    if (msg.length < 96) return;

    const magic = msg.toString('ascii', 0, 4);
    if (magic !== 'STAT') return;

    const version   = msg[4];
    const version2  = msg[5];
    const num_ch    = msg[6];
    const interval  = msg.readUInt16BE(8);

    // STAT v0x05 layout (Big-Endian):
    // 10..13 und, 14..17 ovw, 18..21 late, 22..25 early
    // 26..57 peaks (16x u16)
    // 58..65 ptp_sec u64
    // 66..69 ptp_ns  u32
    // 70 buffer_fill u8
    // 72..75 spdif_samples u32
    // 76..79 xfade_main u32
    // 80..83 xfade_extra u32
    // 84..85 xfade_dup u16
    // 86..87 xfade_zero u16
    // 88..91 samples_read u32
    // 92..95 samples_written u32
    // 96..127 gains (16x u16, bits[9:0])
    // 128..143 pans (16x s8)
    // 144..145 mute_mask u16
    // 146..147 solo_mask u16
    // 148..403 names (16x16)

    if (version < 0x05 || version2 > 0x00 || msg.length < 404) {
        // ignore older payloads here (this server version targets v0x05)
        return;
    }

    const und   = msg.readUInt32BE(10);
    const ovw   = msg.readUInt32BE(14);
    const late  = msg.readUInt32BE(18);
    const early = msg.readUInt32BE(22);

    const peaks = new Array(16);
    for (let i=0; i<16; i++) peaks[i] = msg.readUInt16BE(26 + i*2);

    const ptpSec = msg.readBigUInt64BE(58);          // BigInt
    const ptpNs  = msg.readUInt32BE(66) >>> 0;       // number (0..999_999_999)
    const bufferFill = msg.readUInt8(70);

    const spdifSamples = msg.readUInt32BE(72) >>> 0;
    const xfadeMainCnt = msg.readUInt32BE(76) >>> 0;
    const xfadeExtraCnt= msg.readUInt32BE(80) >>> 0;
    const xfadeDupCnt  = msg.readUInt16BE(84) >>> 0;
    const xfadeZeroCnt = msg.readUInt16BE(86) >>> 0;

    const samplesRead  = msg.readUInt32BE(88) >>> 0;
    const samplesWritten = msg.readUInt32BE(92) >>> 0;

    const newGains = new Array(16);
    for (let i=0; i<16; i++) newGains[i] = msg.readUInt16BE(96 + i*2) & 0x03FF;

    const newPans = new Array(16);
    for (let i=0; i<16; i++) newPans[i] = msg.readInt8(128 + i);

    const muteMask = msg.readUInt16BE(144) & 0xFFFF;
    const soloMaskRx = msg.readUInt16BE(146) & 0xFFFF;

    const newNames = new Array(16);
    for (let i=0; i<16; i++) newNames[i] = readAsciiZ(msg, 148 + i*16, 16);

    // Update global state from FPGA status (authoritative for UI refresh)
    gains = newGains;
    pans  = newPans;
    names = newNames;
    mutes = new Array(16).fill(false).map((_,i)=> ((muteMask>>i)&1)===1);
    solos = new Array(16).fill(false).map((_,i)=> ((soloMaskRx>>i)&1)===1);
    soloMask = soloMaskRx;

    // Broadcast to WS clients
    broadcast({
        type: 'status',
        version,
        numCh: num_ch,
        intervalMs: interval,
        statUnderrun: und,
        statOverwrite: ovw,
        statLate: late,
        statEarly: early,
        peaks,
        ptpSec: ptpSec.toString(),
        ptpNs,
        bufferFill,
        spdifSamples,
        xfadeMainCnt,
        xfadeExtraCnt,
        xfadeDupCnt,
        xfadeZeroCnt,
        samplesRead,
        samplesWritten,
        gains: newGains,
        pans: newPans,
        muteMask,
        mutes,
        solos,
        soloMask: soloMaskRx,
        names: newNames
    });

    // CSV log row
    // logfile expects ptp time as sec + ns (already ns)
    const ptpNsBig = BigInt(ptpNs);
    appendCsvRow([
        nowIso(),
        ptpSec.toString(),
        ptpNsBig.toString(),
        samplesRead,
        samplesWritten,
        late,
        early,
        ovw,
        und,
        bufferFill,
        spdifSamples,
        xfadeMainCnt,
        xfadeExtraCnt,
        xfadeDupCnt,
        xfadeZeroCnt
    ]);
});

udpStatusSock.bind(STATUS_PORT);
