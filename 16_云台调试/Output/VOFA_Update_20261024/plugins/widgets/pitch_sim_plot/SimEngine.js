/* Display-only model. Never writes host channels, serial data, control or scores. */
function createEngine(options) {
    options = options || {};
    var duration = options.durationMs || 240;
    var enabled = true, active = false, previousFailed = false;
    var lastId = null, lastPhase = 0, lastReal = null, lastAt = null;
    var startAngle = 0, simTarget = 0, runAt = null, lastResult = null;
    var points = [], pending = "", pitchPrefix = false, gaps = 0;
    var accepted = 0, simSamples = 0;
    function snapshot() {
        return {enabled:enabled, active:active, previousFailed:previousFailed,
            realAngle:lastReal, displayAngle:lastReal !== null && points.length ? points[points.length-1].angle : null,
            target:points.length ? points[points.length-1].target : null,
            lastRunAt:runAt, lastResult:lastResult, accepted:accepted,
            simSamples:simSamples, gaps:gaps, pointCount:points.length};
    }
    function gap() {
        active = false; previousFailed = false; lastId = null; lastPhase = 0;
        lastReal = null; lastAt = null; pending = ""; pitchPrefix = false; gaps++;
    }
    function setEnabled(value) {
        enabled = value === true;
        if (!enabled) active = false;
    }
    function feedFrame(values, at) {
        if (!values || values.length !== 65 || !isFinite(at) || at < 0) return false;
        for (var i=0; i<values.length; ++i)
            if (typeof values[i] !== "number" || !isFinite(values[i])) return false;
        var id=values[24], phase=values[25], real=values[0], target=values[1];
        if (id < 0 || Math.floor(id)!==id || phase<0 || phase>3 || Math.floor(phase)!==phase ||
            Math.abs(real)>360 || target < -18 || target > 33 || values[28]<0) return false;
        if (lastAt !== null && at < lastAt) gap();
        var newRun = phase === 1 && (lastId !== id || lastPhase !== 1);
        if (newRun) {
            active = enabled && previousFailed && lastId !== null && lastId !== id;
            startAngle = lastReal === null ? real : lastReal;
            simTarget = target; runAt = at; previousFailed = false;
        }
        if (active && (phase !== 1 || target !== simTarget)) active = false;
        if (phase === 2 && (lastPhase !== 2 || lastId !== id)) {
            previousFailed = values[27] < 0 || values[27] > 300 || values[28] > 0.2;
            lastResult = {id:id, settleMs:values[27], overshootDeg:values[28], failed:previousFailed};
        } else if (phase === 3 || phase === 0) {
            active = false; previousFailed = false;
        }
        var angle = real;
        if (active) {
            var s = Math.max(0,Math.min(1,(at-runAt)/duration));
            var blend = s*s*s*(10+s*(-15+6*s));
            angle = startAngle + (simTarget-startAngle)*blend;
            simSamples++;
        }
        points.push({at:at, angle:angle, target:target, real:real, sim:active});
        if (points.length>1600) points.splice(0,points.length-1600);
        lastId=id; lastPhase=phase; lastReal=real; lastAt=at; accepted++;
        return true;
    }
    function numeric(text, at) {
        var parts=text.split(',');
        if (parts.length!==65) return false;
        var values=[];
        for (var i=0;i<parts.length;++i) {
            if (!/^[ \t]*[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?[ \t]*$/.test(parts[i])) return false;
            values.push(Number(parts[i]));
        }
        return feedFrame(values,at);
    }
    function feed(text, at) {
        if (typeof text!=="string" || !isFinite(at)) return 0;
        pending+=text;
        if (pending.length>65536) { gap(); return 0; }
        var before=accepted, lines=pending.split('\n'); pending=lines.pop();
        for (var i=0;i<lines.length;++i) {
            var line=lines[i].replace(/\r$/, '').replace(/^\[\d{2}:\d{2}:\d{2}\.\d{3}\] /,'');
            if (!line) continue;
            if (line.indexOf('pitch:')===0) {
                var rest=line.slice(6).replace(/^[ \t]+|[ \t]+$/g,'');
                pitchPrefix=rest.length===0;
                if (rest) numeric(rest,at);
            } else if (pitchPrefix) {
                pitchPrefix=false; numeric(line,at);
            }
        }
        return accepted-before;
    }
    function clearPoints() { points=[]; simSamples=0; }
    return {feed:feed,feedFrame:feedFrame,snapshot:snapshot,setEnabled:setEnabled,
        gap:gap,clearPoints:clearPoints,points:function(){return points.slice(0);}};
}
if (typeof module !== 'undefined' && module.exports) module.exports={createEngine:createEngine};
