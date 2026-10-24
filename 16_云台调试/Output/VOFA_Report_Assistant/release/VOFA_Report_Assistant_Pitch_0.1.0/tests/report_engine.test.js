/* Offline protocol tests. No serial device, VOFA instance, or firmware is touched. */
"use strict";
const fs = require("fs");
const path = require("path");
const vm = require("vm");
const assert = require("assert");
const source = fs.readFileSync(path.join(__dirname, "../plugins/widgets/report_assistant/ReportEngine.js"), "utf8");
const sandbox = {};
vm.createContext(sandbox);
vm.runInContext(source, sandbox, { filename: "ReportEngine.js" });
let passed = 0;
let assertions = 0;
function eq(a, b, message) { assertions++; assert.strictEqual(a, b, message); }
function yes(v, message) { assertions++; assert.ok(v, message); }
function test(name, fn) {
    try { fn(); passed++; process.stdout.write("PASS " + name + "\n"); }
    catch (err) { process.stderr.write("FAIL " + name + "\n" + err.stack + "\n"); process.exitCode = 1; }
}
function engine(options) { return sandbox.createEngine(options); }
function last(e) { return e.reports[e.reports.length - 1]; }
function send(e, command, at = 0) { yes(e.start(command, at).ok); return last(e); }
function rx(e, text, at = 1) { yes(e.feed(text + "\r\n", at).ok); }
const check = "P|CHECK|PITCH_4|QUERY|NEW_FEEDBACK|CTRL|CASCADE|FAULT|0";
const busy = "P|BUSY|CHECK|QUERY|CC|FC_SENT|0|FD_SENT|0|READ_ONLY_MONITOR|1";
const guard = "P|GUARD|V1|TRACE|12|P_FAULT|0";
const step = "P|STEP|ID|12|STATE|DONE|TARGET|30.000|VALID|1|PASS|1|FAULT|0";

test("ES5 source avoids newer JavaScript syntax", () => {
    eq(/\b(?:let|const|class)\s|=>|`|\?\.|\.\.\./.test(source.replace(/\/\*[\s\S]*?\*\//g, "").replace(/\/\/[^\n]*/g, "")), false);
});
test("only exact finite numeric CSV telemetry is suppressed", () => {
    const e = engine();
    ["pitch:1,2,3", "gimbal:-1,+2.0,.3,4.,1e-2", "feeder:0, 2 "].forEach(s => rx(e, s));
    const retained = ["pitch:", "pitch:1,", "pitch:NaN", "pitch:Infinity", "pitch:1e999", "pitch:0x10", "pitch:1,hello", "pitch:1 2", "Pitch:1,2", "g:1,2", " pitch:1,2", "pitch:1|2", "P|PID|POS|1,2,3", "status text"];
    retained.forEach(s => rx(e, s));
    eq(e.stats.telemetryLines, 3); eq(e.events.length, retained.length);
    retained.forEach((s, i) => eq(e.events[i].text, s));
});
test("200 Hz telemetry interleaves CHECK without losing report or event order", () => {
    const e = engine(); const r = send(e, "PITCH CHECK");
    for (let i = 0; i < 1000; i++) {
        rx(e, "pitch:" + i + ",1.5,0,0", i * 5 + 1);
        if (i === 2) rx(e, busy, i * 5 + 1);
        if (i === 4) rx(e, "R|RX|EPOCH|3", i * 5 + 1);
        if (i === 13) rx(e, "P|REMOTE_STEP|SEQ|2|RESULT|BLOCKED", i * 5 + 1);
        if (i === 30) rx(e, check, i * 5 + 1);
    }
    eq(e.stats.telemetryLines, 1000); eq(r.status, "complete"); eq(r.lines.length, 2);
    eq(r.lines[0].text, busy); eq(r.lines[1].text, check);
    eq(e.events.length, 2); yes(e.events[0].seq < e.events[1].seq); eq(e.pendingId, null);
});
test("CRLF and LF are assembled at every possible two-chunk split", () => {
    const wire = busy + "\r\npitch:1,2\n" + check + "\r\n";
    for (let i = 0; i <= wire.length; i++) {
        const e = engine(); const r = send(e, "PITCH CHECK");
        e.feed(wire.substring(0, i), 1); e.feed(wire.substring(i), 2);
        eq(r.status, "complete", "split " + i); eq(r.lines.length, 2); eq(e.stats.telemetryLines, 1); eq(e.events.length, 0);
    }
});
test("single-character fragmented multi-line stream has first-byte timestamps", () => {
    const e = engine(); const r = send(e, "PITCH CHECK");
    for (const [i, c] of Array.from(busy + "\r\n" + check + "\n").entries()) e.feed(c, i + 1);
    eq(r.status, "complete"); eq(r.lines[0].at, 1); eq(r.lines[1].at, busy.length + 3);
});
test("unterminated text does not match early", () => {
    const e = engine(); const r = send(e, "PITCH PID");
    e.feed("P|PID|POS|1,2,3", 1); eq(r.lines.length, 0); eq(r.status, "waiting");
    e.feed("\n", 2); eq(r.status, "complete");
});
test("line begun before a command cannot be claimed by that command", () => {
    const e = engine(); e.feed("P|PID|", 1); const r = send(e, "PITCH PID", 2);
    e.feed("POS|1,2,3\n", 3); eq(r.lines.length, 0); eq(e.events.length, 1); eq(r.status, "waiting");
});
test("CHECK accepts only its final QUERY state", () => {
    const e = engine(); const r = send(e, "PITCH CHECK");
    rx(e, "P|CHECK|QUERY|CC"); eq(r.lines.length, 0);
    rx(e, busy); eq(r.status, "waiting");
    rx(e, check); eq(r.status, "complete");
});
test("device CHECK TIMEOUT is terminal error, never success", () => {
    const e = engine(); const r = send(e, "PITCH CHECK");
    rx(e, "P|CHECK|V1|QUERY|TIMEOUT|FAULT|0"); eq(r.status, "error"); eq(r.deviceError, true);
});
test("STATUS is SNAPSHOT_ONLY and cannot finish CHECK", () => {
    const e = engine(); const r = send(e, "PITCH CHECK");
    rx(e, "P|STATUS|V1|QUERY|SNAPSHOT_ONLY|FAULT|0"); eq(r.status, "waiting"); eq(e.events.length, 1);
    e.endWait(2); const s = send(e, "PITCH STATUS", 3);
    rx(e, "P|STATUS|V1|QUERY|NEW_FEEDBACK", 4); eq(s.status, "waiting");
    rx(e, "P|STATUS|V1|QUERY|SNAPSHOT_ONLY", 5); eq(s.status, "complete");
});
test("same-axis and cross-axis unrelated errors remain independent events", () => {
    const e = engine(); const r = send(e, "PITCH CHECK");
    ["Y|ERROR|CHECK|FAULT", "P|ERROR|HOME_REQUIRES_ARMED", "P|ERROR|STEP_OUTSIDE_MECHANICAL_RANGE"].forEach(s => rx(e, s));
    eq(r.status, "waiting"); eq(e.events.length, 3);
    rx(e, "P|ERROR|CHECK|QUERY_UNAVAILABLE"); eq(r.status, "error");
});
test("FAULT zero is healthy, nonzero FAULT and explicit errors are highlighted", () => {
    const e = engine(); ["P|CHECK|FAULT|0", "P|GUARD|P_FAULT|0", "P|STATE|FAULT|32", "P|STATE|FAULT|0x10", "C|UART_FAULT|EPOCH|1"].forEach(s => rx(e, s));
    eq(e.events[0].level, "info"); eq(e.events[1].level, "warning");
    eq(e.events[2].level, "error"); eq(e.events[3].level, "error"); eq(e.events[4].level, "error");
});
test("READY ACK waits, then its own READY completes", () => {
    const e = engine(); const r = send(e, "PITCH READY");
    rx(e, "P|BUSY|READY|QUERY|CC|FC_SENT|0"); eq(r.status, "waiting");
    rx(e, "P|GUARD|TRACE|5"); eq(r.status, "waiting");
    rx(e, "P|OK|READY|ZERO_SET|1"); eq(r.status, "complete"); eq(r.lines.length, 2);
});
test("READY ALREADY_READY completes without pretending to enable motor", () => {
    const e = engine(); const r = send(e, "PITCH READY");
    rx(e, "P|OK|ALREADY_READY|ACTIVE|0|FC_SENT|0"); eq(r.status, "complete");
});
test("BUSY USE_PITCH_OFF rejection is terminal instead of false preparation", () => {
    ["CHECK", "READY"].forEach(action => {
        const e = engine(); const r = send(e, "PITCH " + action);
        rx(e, "P|BUSY|" + action + "|USE_PITCH_OFF_TO_CANCEL"); eq(r.status, "error");
    });
});
test("PITCH RESULT cannot distinguish unsolicited identical STEP", () => {
    const e = engine(); const r = send(e, "PITCH RESULT");
    rx(e, "pitch:1,2"); rx(e, step);
    eq(r.status, "waiting"); eq(r.lines[0].uncertain, true); eq(r.candidateCount, 1);
    eq(e.events[0].text, step); e.tick(4000); eq(r.status, "uncertain"); eq(e.pendingId, null);
});
test("PITCH GUARD and REMOTE RX stay explicitly uncertain", () => {
    [["PITCH GUARD", guard], ["REMOTE RX", "R|RX|EPOCH|7"]].forEach(pair => {
        const e = engine(); const r = send(e, pair[0]); rx(e, pair[1]);
        eq(r.lines[0].uncertain, true); eq(r.status, "waiting"); e.endWait(2); eq(r.status, "uncertain");
    });
});
test("multiple uncertain candidates retain order rather than overwrite", () => {
    const e = engine(); const r = send(e, "PITCH GUARD");
    rx(e, guard, 2); rx(e, "P|GUARD|TRACE|13", 3); eq(r.lines.length, 2);
    eq(r.lines[0].at, 2); eq(r.lines[1].at, 3); eq(r.candidateCount, 2);
});
test("STEP requires its ACK, target and newer id; completion remains candidly uncertain", () => {
    const e = engine(); rx(e, "P|STEP|ID|11|STATE|DONE|TARGET|30", 0);
    const r = send(e, "PITCH STEP 30", 1);
    rx(e, step, 2); eq(r.lines.length, 0);
    rx(e, "P|OK|STEP_STARTED|TARGET|-30|WINDOW_MS|5000", 3); eq(r.lines.length, 0);
    rx(e, "P|OK|STEP_STARTED|TARGET|30|WINDOW_MS|5000", 4); eq(r.status, "waiting");
    rx(e, "P|STEP|ID|11|STATE|DONE|TARGET|30", 5); eq(r.status, "waiting");
    rx(e, "P|STEP|ID|12|STATE|DONE|TARGET|-30", 6); eq(r.status, "waiting");
    rx(e, step, 5004); eq(r.status, "uncertain"); eq(r.lines.length, 2); eq(r.deviceError, false);
});
test("STEP ABORTED is terminal uncertain with a device error flag", () => {
    const e = engine(); const r = send(e, "PITCH STEP -30");
    rx(e, "P|OK|STEP_STARTED|TARGET|-30|WINDOW_MS|5000");
    rx(e, "P|STEP|ID|1|STATE|ABORTED|TARGET|-30|REASON|STOP", 200);
    eq(r.status, "uncertain"); eq(r.deviceError, true);
});
test("STEP without ID cannot fabricate completion", () => {
    const e = engine(); const r = send(e, "PITCH STEP");
    rx(e, "P|OK|STEP_STARTED|TARGET|30"); rx(e, "P|STEP|STATE|DONE|TARGET|30"); eq(r.status, "waiting");
});
test("malformed nonfinite or missing STEP ACK target cannot authorize a later candidate", () => {
    ["NaN", "Infinity", "1e999", null].forEach(target => {
        const e = engine(); const r = send(e, "PITCH STEP");
        rx(e, "P|OK|STEP_STARTED" + (target === null ? "" : "|TARGET|" + target));
        rx(e, step); eq(r.status, "waiting"); eq(r.lines.length, 0);
    });
});
test("overflowed unsolicited step ID cannot poison later valid step matching", () => {
    const e = engine(); rx(e, "P|STEP|ID|" + "9".repeat(400) + "|STATE|DONE|TARGET|30", 0);
    const r = send(e, "PITCH STEP", 1); rx(e, "P|OK|STEP_STARTED|TARGET|30", 2); rx(e, step, 3);
    eq(r.status, "uncertain"); eq(r.lines.length, 2);
});
test("parameter write replies and relevant errors match only intended command", () => {
    const examples = [["PITCH PKP 2", "P|OK|PARAMETER"], ["PITCH POS 1 2 3", "P|OK|POS"], ["PITCH KG 0.1", "P|OK|KG_SET|KG|0.1"], ["PITCH GHORIZ", "P|OK|HORIZONTAL_SET|G_CAL|1"], ["PITCH ZERO", "P|OK|SOFTWARE_ZERO"], ["PITCH BRAKE ON", "P|OK|BRAKE|ON"], ["PITCH OFF", "P|OK|OFF_REQUESTED"], ["PITCH HOME", "P|OK|HOME|TARGET|0"]];
    examples.forEach(pair => { const e = engine(); const r = send(e, pair[0]); rx(e, pair[1]); eq(r.status, "complete"); });
    const e = engine(); const r = send(e, "PITCH VKP -1"); rx(e, "P|ERROR|PID_RANGE_OR_FORMAT"); eq(r.status, "error");
});
test("all Pitch read-only default commands map to their real firmware reply", () => {
    const examples = [["PITCH", "P|STATE|V1"], ["PITCH PID", "P|PID|POS|1,2,3"], ["PITCH FF", "P|FF|KG|0"], ["PITCH BRAKE", "P|BRAKE|MODE|TERMINAL_AND_RETURN"], ["PITCH SHAPE", "P|SHAPE|MS|0"], ["PITCH SHAPEV", "P|SHAPEV|1"], ["PITCH HELP", "P|HELP|READ|PITCH"], ["PITCH ?", "P|HELP|READ|PITCH"]];
    examples.forEach(pair => { const e = engine(); const r = send(e, pair[0]); rx(e, pair[1]); eq(r.status, "complete"); });
});
test("diagnostic DM uses matching BEGIN/DATA/DONE despite interleaving", () => {
    const e = engine(); const r = send(e, "READ DM 3");
    rx(e, "D|BEGIN|DM|4"); rx(e, "D|DONE|DM|3"); eq(r.status, "waiting"); eq(r.candidateCount, 1);
    rx(e, "D|BEGIN|DM|3"); rx(e, "D|DM|4|AAA"); rx(e, "P|GUARD|TRACE|1"); rx(e, "D|DM|3|010203");
    rx(e, "D|DONE|VESC|1|3"); eq(r.status, "waiting"); rx(e, "D|DONE|DM|3");
    eq(r.status, "uncertain"); eq(r.lines.length, 4); eq(e.events.length, 5);
});
test("diagnostic VESC keys node and field, keeps multiple payload fragments", () => {
    const e = engine(); const r = send(e, "READ VESC 2 8");
    rx(e, "D|BEGIN|VESC|2|8"); rx(e, "D|VESC|2|9|6|0|ABC");
    rx(e, "D|VESC|2|8|6|0|0123"); rx(e, "pitch:1,2"); rx(e, "D|VESC|2|8|6|2|4567");
    rx(e, "D|DONE|VESC|3|8"); eq(r.status, "waiting"); rx(e, "D|DONE|VESC|2|8");
    eq(r.status, "complete"); eq(r.lines.length, 4); eq(e.events.length, 2);
});
test("unkeyed diagnostics error is uncertain until own BEGIN established", () => {
    let e = engine(), r = send(e, "READ DM 1"); rx(e, "D|ERROR|BUSY"); eq(r.status, "uncertain");
    e = engine(); r = send(e, "READ DM 1"); rx(e, "D|BEGIN|DM|1"); rx(e, "D|ERROR|TIMEOUT"); eq(r.status, "error");
});
test("diagnostic STATUS multiline closes only D DONE STATUS", () => {
    const e = engine(); const r = send(e, "STATUS");
    rx(e, "D|STATUS|1|2|3"); rx(e, "D|CAN|1|0|123|8|AABB|2|3"); rx(e, "D|DONE|DM|1");
    eq(r.status, "waiting"); rx(e, "D|DONE|STATUS"); eq(r.status, "complete"); eq(r.lines.length, 3);
});
test("one pending command only and no retry creation on timeout", () => {
    const e = engine(); const r = send(e, "PITCH CHECK");
    eq(e.start("PITCH PID", 1).ok, false); eq(e.reports.length, 1);
    e.tick(4000); eq(r.status, "timeout"); e.tick(8000); eq(e.reports.length, 1);
});
test("timeout is checked before arriving delayed reply, including absent timer", () => {
    const e = engine(); const r = send(e, "PITCH CHECK"); rx(e, check, 4001);
    eq(r.status, "timeout"); eq(r.lines.length, 0); eq(e.events[0].text, check);
});
test("retry after unresolved identical command cannot definitively claim late reply", () => {
    const e = engine(); send(e, "PITCH CHECK"); e.tick(4000);
    const r = send(e, "PITCH CHECK", 4001); rx(e, check, 4002);
    eq(r.status, "uncertain"); eq(r.lines[0].uncertain, true);
    const s = send(e, "PITCH PID", 4003); rx(e, "P|PID|POS|1,2,3", 4004); eq(s.status, "complete");
});
test("stale correlation guard expires after its bounded horizon", () => {
    const e = engine(); send(e, "PITCH CHECK"); e.endWait(1);
    const r = send(e, "PITCH CHECK", 30002); rx(e, check, 30003); eq(r.status, "complete");
});
test("ending wait sends nothing and delayed reply stays visible", () => {
    const e = engine(); const r = send(e, "PITCH CHECK"); e.endWait(1); rx(e, check, 2);
    eq(r.status, "ended"); eq(r.lines.length, 0); eq(e.events[0].text, check); eq(e.reports.length, 1);
});
test("explicit send failure releases pending and preserves a reason", () => {
    const e = engine(); const r = send(e, "PITCH PID"); e.failSend("offline", 1);
    eq(r.status, "error"); eq(e.pendingId, null); yes(r.warnings[0].includes("offline"));
});
test("line overflow bounded and never interpreted as complete reply", () => {
    const e = engine({maxLineLength:64}); const r = send(e, "PITCH PID");
    e.feed("P|PID|" + "a".repeat(100000), 1); eq(e.events.length, 0);
    e.feed("\n", 2); eq(r.status, "waiting"); eq(e.events.length, 1); eq(e.stats.truncatedLines, 1);
    yes(e.events[0].text.length < 120); yes(e.events[0].truncated); yes(e.warnings.length > 0);
    rx(e, "P|PID|POS|1,2,3", 3); eq(r.status, "uncertain");
});
test("event count and character caps warn and preserve retained order", () => {
    const e = engine({maxEvents:3,maxEventChars:256});
    for (let i = 0; i < 10; i++) rx(e, "line" + i);
    eq(e.events.length, 3); eq(e.events[0].text, "line7"); eq(e.stats.droppedEvents, 7); yes(e.warnings.length > 0);
    rx(e, "x".repeat(250)); yes(e.events.reduce((n,l) => n+l.text.length, 0) <= 256);
});
test("report line caps warn without discarding terminal status", () => {
    const e = engine({maxLinesPerReport:2}); const r = send(e, "READ DM 3");
    rx(e, "D|BEGIN|DM|3"); rx(e, "D|DM|3|1111"); rx(e, "D|DM|3|2222"); rx(e, "D|DONE|DM|3");
    eq(r.status, "complete"); eq(r.lines.length, 2); eq(r.lineDrops, 2); yes(r.warnings.length > 0);
    eq(r.lines[0].text, "D|DM|3|2222");
});
test("report character and global history bounds hold", () => {
    const e = engine({maxCharsPerReport:256,maxTotalReportChars:512,maxReports:100});
    for (let i = 0; i < 10; i++) { send(e, "PITCH PID", i*10); rx(e, "P|PID|" + "a".repeat(200), i*10+1); }
    yes(e.reports.length <= 2); yes(e.stats.droppedReports >= 8);
    yes(e.reports.reduce((n,r) => n+r.charCount, 0) <= 512);
});
test("global report bound also holds with a single unusually large report", () => {
    const e = engine({maxTotalReportChars:512}); const r = send(e, "PITCH PID");
    rx(e, "P|PID|" + "a".repeat(2000));
    yes(r.charCount <= 512); eq(r.lineDrops, 1); yes(r.warnings.length > 0); eq(r.status, "complete");
});
test("recent 100 reports are retained in chronological order", () => {
    const e = engine();
    for (let i = 0; i < 110; i++) { send(e, "PITCH PID", i*2); rx(e, "P|PID|POS|" + i, i*2+1); }
    eq(e.reports.length, 100); eq(e.reports[0].id, 11); eq(last(e).id, 110); eq(e.stats.droppedReports, 10);
});
test("gap marker prevents partial-line splicing and invalidates confidence", () => {
    const e = engine(); const r = send(e, "PITCH CHECK");
    e.feed("P|CHE", 1); e.noteGap("adapter overrun", 2); e.feed("CK|QUERY|NEW_FEEDBACK\n", 3);
    eq(r.status, "waiting"); eq(e.events.length, 3); eq(e.stats.gaps, 1);
    rx(e, check, 4); eq(r.status, "uncertain"); yes(r.warnings.some(s => s.includes("adapter overrun")));
});
test("editable group rules always retain malformed or nonnumeric text", () => {
    const e = engine(); yes(e.setRules(["custom", "custom"]).ok); eq(e.getRules().length, 1);
    rx(e, "custom:1,2"); rx(e, "custom:ERROR"); rx(e, "pitch:1,2");
    eq(e.stats.telemetryLines, 1); eq(e.events.length, 2);
    eq(e.setRules([".*"]).ok, false); eq(e.getRules()[0], "custom");
    yes(e.setRules([]).ok); rx(e, "custom:1,2"); eq(e.events.length, 3);
});
test("telemetry only does not schedule UI redraws", () => {
    const e = engine(); eq(e.feed("pitch:1,2\n", 1).changed, false);
    eq(e.stats.telemetryLines, 1); eq(e.feed("hello\n", 2).changed, true);
});
test("serializer roundtrip preserves command, original text, time, seq and uncertainty", () => {
    const e = engine(); send(e, "PITCH GUARD", 100); rx(e, guard, 101); e.endWait(102);
    const f = engine(); yes(f.restore(e.serialize()).ok); const r = f.reports[0];
    eq(r.command, "PITCH GUARD"); eq(r.sentAt, 100); eq(r.lines[0].at, 101); eq(r.lines[0].text, guard);
    eq(r.status, "uncertain"); eq(r.lines[0].uncertain, true); eq(f.pendingId, null);
    yes(f.exportText(r.id).includes("归属待确认"));
});
test("restart interrupts pending and never reissues it", () => {
    const e = engine(); send(e, "PITCH CHECK", 100); rx(e, busy, 101);
    const f = engine(); yes(f.restore(e.serialize()).ok); eq(f.reports[0].status, "interrupted"); eq(f.pendingId, null);
    const r = send(f, "PITCH CHECK", 102); rx(f, check, 103); eq(r.status, "uncertain");
});
test("invalid serialized JSON and schema leave active data untouched", () => {
    const e = engine(); const r = send(e, "PITCH PID");
    ["{", "null", JSON.stringify({version:2,reports:[],events:[]}), JSON.stringify({version:1,reports:{},events:[]})].forEach(data => {
        eq(e.restore(data).ok, false); eq(e.pendingId, r.id); eq(e.reports.length, 1);
    });
});
test("corrupt persisted record fields are sanitized without executable matcher reuse", () => {
    const e = engine(); send(e, "PITCH CHECK"); const data = JSON.parse(e.serialize());
    data.reports.push({id:999,command:"PITCH\nPID",sentAt:0,lines:[]});
    data.reports[0]._match = {kind:"read",axis:"Y",reply:"PID"}; data.reports[0].lineDrops = Infinity;
    data.reports[0].lines = [{text:"bad",at:1e100,seq:1},{text:"valid",at:1,seq:2,level:"hack"}];
    data.stats.gaps = Infinity; const f = engine(); yes(f.restore(data).ok);
    eq(f.reports.length, 1); eq(f.reports[0].lines.length, 1); eq(f.reports[0].lineDrops, 0); eq(f.stats.gaps, 0);
    eq(f.reports[0]._match.kind, "check"); yes(f.warnings.length > 0);
});
test("restoring oversized history obeys caps and announces loss", () => {
    const data = {version:1,reports:[],events:[]};
    for (let i = 1; i <= 10; i++) data.reports.push({id:i,command:"PITCH PID",sentAt:i,status:"complete",lines:[{text:"P|PID|POS|1,2,3",at:i,seq:i}]});
    for (let i = 0; i < 10; i++) data.events.push({text:"x".repeat(1000),at:i,seq:i});
    const e = engine({maxReports:3,maxEvents:2,maxLineLength:64,maxEventChars:256}); yes(e.restore(data).ok);
    eq(e.reports.length, 3); eq(e.reports[0].id, 8); yes(e.events.length <= 2); yes(e.stats.droppedReports >= 7);
    yes(e.stats.truncatedLines > 0); yes(e.warnings.length > 0);
});
test("export is plain text and maintains original receive order", () => {
    const e = engine(); rx(e, "<b>not HTML</b>", 1); rx(e, "汉字报告", 2);
    const out = e.exportText("events"); yes(out.includes("<b>not HTML</b>")); yes(out.includes("汉字报告"));
    yes(out.indexOf("<b>") < out.indexOf("汉字")); eq(e.exportText(999), "");
});
test("invalid input and multiline command cannot accidentally send multiple commands", () => {
    const e = engine(); ["", " ", "PITCH\nPID", "PITCH\rPID", "PITCH\u0000PID", "x".repeat(64), null].forEach(cmd => eq(e.start(cmd, 0).ok, false));
    eq(e.reports.length, 0); eq(e.feed({}, 1).ok, false); eq(e.endWait(2).ok, false);
});
test("Yaw read, async READY and isolated protocol remain supported", () => {
    const e = engine(); let r = send(e, "YAW CHECK"); rx(e, "Y|CHECK|HEALTH|1"); eq(r.status, "complete");
    r = send(e, "YAW READY", 2); rx(e, "Y|BUSY|READY", 3); eq(r.status, "waiting");
    rx(e, "P|OK|READY", 4); eq(r.status, "waiting"); rx(e, "Y|OK|READY", 5); eq(r.status, "complete");
});
test("view and telemetry command acknowledgement is precise", () => {
    [["PITCH VIEW","P|VIEW|PITCH|GROUP|pitch"],["YAW VIEW","Y|VIEW|YAW|GROUP|gimbal"],["TELEM PITCH","P|VIEW|PITCH|GROUP|pitch"],["TELEM ON","C|OK|TELEM|ON"]].forEach(pair => {
        const e = engine(); const r = send(e, pair[0]); rx(e, pair[1]); eq(r.status, "complete");
    });
});
test("independent engines do not share mutable state", () => {
    const a = engine(), b = engine(); send(a,"PITCH PID"); a.setRules(["custom"]); rx(a,"P|PID|POS|1,2,3");
    eq(b.reports.length,0); eq(b.getRules()[0],"gimbal"); eq(b.stats.receivedLines,0);
});
process.stdout.write("\n" + passed + " test cases passed; " + assertions + " assertions.\n");
if (process.exitCode) process.exit(process.exitCode);
