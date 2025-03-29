function kalmanFilter(entry, eMea, eEst, q, lastEstimate, safeDivisionUnit = 1e-10) {
    let kalmanGain = eEst / (eEst + eMea + safeDivisionUnit);
    let currentEstimate = lastEstimate + kalmanGain * (entry - lastEstimate);
    let newEEst = (1 - kalmanGain) * eEst + Math.abs(lastEstimate - currentEstimate) * q;
    return [currentEstimate, newEEst];
}

function emaFilter(entry, length, prev, step, safeDivisionUnit = 1e-10) {
    let alpha = 2 / (length + 1 + safeDivisionUnit);
    return [step >= length ? entry * alpha + prev * (1 - alpha) : entry, entry, step];
}

function smaFilter(entry, length, accumulated, safeDivisionUnit = 1e-10) {
    accumulated.push(entry);
    accumulated = accumulated.slice(-length);
    return [accumulated.length < length ? entry : accumulated.reduce((acc, val) => acc + val, 0) / (length + safeDivisionUnit), accumulated];
}

function medianFilter(entry, size, buffer, count, safeDivisionUnit = 1e-10) {
    if (buffer.length !== size) {
        buffer = [];
        for (let i = 0; i < size; i++) buffer[i] = 0;
    }
    buffer[count] = entry;
    if ((count < size - 1) && (buffer[count] > buffer[count + 1])) {
        for (let i = count; i < size - 1; i++) {
            if (buffer[i] > buffer[i + 1]) {
                const temp = buffer[i];
                buffer[i] = buffer[i + 1];
                buffer[i + 1] = temp;
            }
        }
    } else {
        if ((count > 0) && (buffer[count - 1] > buffer[count])) {
            for (let i = count; i > 0; i--) {
                if (buffer[i] < buffer[i - 1]) {
                    const temp = buffer[i];
                    buffer[i] = buffer[i - 1];
                    buffer[i - 1] = temp;
                }
            }
        }
    }
    count++;
    if (count >= size) count = 0;
    return [buffer[Math.floor(size / 2)], buffer, count];
}

function clip(value, min, max) {
    if (value > max) {
        value = max;
    } else if (value < min) {
        value = min;
    }
    return value;
}

function onErrorProportional(target, entry) {
    return target - entry;
}

function onRateProportional(entry, initialEntry) {
    return entry - initialEntry;
}

function onErrorIntegral(prevIntegral, error, time) {
    return prevIntegral + error * time;
}

function onWindowIntegral(window, size) {
    size = window.length <= size ? 0 : size;
    return window.slice(-size).reduce((acc, val) => acc + val);
}

function strToBool(str) {
    if (typeof(str) === 'string') {
        return str === 'false' ? false : true
    } else if (typeof(str) === 'boolean') {
        return str
    }
    return null
}

module.exports = (RED) => {
    function pidRegulator(config) {
        RED.nodes.createNode(this, config);
        this.on('input', (msg, send, done) => {
            const safeDivisionUnit = 1e-10;
            const contextParams = {
                kp: 'kp',
                ki: 'ki',
                kd: 'kd',
                target: 'target',
                t: 't',
                direction: 'direction',
                clipOutput: 'clipOutput',
                outputMinLimit: 'outputMinLimit',
                outputMaxLimit: 'outputMaxLimit',
                pCalculationMode: 'pCalculationMode',
                clipI: 'clipI',
                iMinLimit: 'iMinLimit',
                iMaxLimit: 'iMaxLimit',
                clipIAsOutput: 'clipIAsOutput',
                iCalculationMode: 'iCalculationMode',
                iWindowSize: 'iWindowSize',
                dFilter1: 'dFilter1',
                dFilter2: 'dFilter2',
                dFilter3: 'dFilter3',
                dFilter4: 'dFilter4',
                dFilterKalmanEMea: 'dFilterKalmanEMea',
                dFilterKalmanQ: 'dFilterKalmanQ',
                dFilterEMALength: 'dFilterEMALength',
                dFilterSMALength: 'dFilterSMALength',
                dFilterMedianSize: 'dFilterMedianSize',
                fallbackValue: 'fallbackValue',
                sendState: 'sendState',
                sendConfig: 'sendConfig',
                force: 'force',
                clearCache: 'clearCache',
                prevP: 'prevP',
                prevI: 'prevI',
                prevD: 'prevD',
                prevError: 'prevError',
                prevOutput: 'prevOutput',
                initialEntry: 'initialEntry',
                iWindow: 'iWindow',
                lastCalculationTime: 'lastCalculationTime',
                dFilterKalmanEEst: 'dFilterKalmanEEst',
                dFilterKalmanLastEstimate: 'dFilterKalmanLastEstimate',
                dFilterEMAPrev: 'dFilterEMAPrev',
                dFilterEMAStep: 'dFilterEMAStep',
                dFilterSMAAccumulated: 'dFilterSMAAccumulated',
                dFilterMedianBuffer: 'dFilterMedianBuffer',
                dFilterMedianCount: 'dFilterMedianCount'
            };
            const nodeContext = this.context();
            const context = {};
            for (let e of nodeContext.keys()) context[e] = nodeContext.get(e);

            function getValue(name, def, allow_override=true) {
                const checkConfig = config === null ? false : true, checkContext = context === null ? false : true, checkMsg = msg === null ? false : true;
                let msgOverride = false
                let target = def;
                if (checkConfig) {
                    let configTarget = config[name];
                    target = configTarget != null && configTarget != undefined ? configTarget : target;
                }
                if (checkContext) {
                    let contextTarget = context[name];
                    target = contextTarget != null && contextTarget != undefined ? contextTarget : target;
                }
                if (checkMsg) {
                    let msgTarget = msg[name];
                    if (msgTarget != null && msgTarget != undefined) {
                        target = msgTarget;
                        msgOverride = true;
                    }
                }
                if (msgOverride && allow_override) {
                    nodeContext.set(name, target)
                    for (let e of nodeContext.keys()) context[e] = nodeContext.get(e);
                }
                return target;
            }

            const pidCoeffs = () => {
                const kp = getValue(contextParams.kp, 0);
                const ki = getValue(contextParams.ki, 0);
                const kd = getValue(contextParams.kd, 0);
                return [Number(kp), Number(ki), Number(kd)];
            };
            const clipOutputConfig = () => {
                const clipOutput = getValue(contextParams.clipOutput);
                const outputMinLimit = getValue(contextParams.outputMinLimit, 0);
                const outputMaxLimit = getValue(contextParams.outputMaxLimit, 0);
                return [strToBool(clipOutput), Number(outputMinLimit), Number(outputMaxLimit)];
            };
            const clipIConfig = () => {
                const clipI = getValue(contextParams.clipI);
                const iMinLimit = getValue(contextParams.iMinLimit, 0);
                const iMaxLimit = getValue(contextParams.iMaxLimit, 0);
                return [strToBool(clipI), Number(iMinLimit), Number(iMaxLimit)];
            };
            const filtersConfig = () => {
                const filter1 = getValue(contextParams.dFilter1, 'none');
                const filter2 = getValue(contextParams.dFilter2, 'none');
                const filter3 = getValue(contextParams.dFilter3, 'none');
                const filter4 = getValue(contextParams.dFilter4, 'none');
                return [ filter1, filter2, filter3, filter4 ];
            };
            const kalmanFilterConfig = () => {
                const eMea = Number(getValue(contextParams.dFilterKalmanEMea, 1));
                const eEst = Number(getValue(contextParams.dFilterKalmanEEst, 1));
                const q = Number(getValue(contextParams.dFilterKalmanQ, 0.01));
                const lastEstimate = Number(getValue(contextParams.dFilterKalmanLastEstimate, 0)); 
                return [ eMea, eEst, q, lastEstimate ];
            };
            const emaFilterConfig = () => {
                const length = Number(getValue(contextParams.dFilterEMALength, 10));
                const prev = Number(getValue(contextParams.dFilterEMAPrev, 0));
                const step = Number(getValue(contextParams.dFilterEMAStep, 0));
                return [ length, prev, step ];
            };
            const smaFilterConfig = () => {
                const length = Number(getValue(contextParams.dFilterSMALength, 10));
                const accumulated = getValue(contextParams.dFilterSMAAccumulated, []);
                return [ length, accumulated ];
            };
            const medianFilterConfig = () => {
                const size = Number(getValue(contextParams.dFilterMedianSize, 10));
                const buffer = getValue(contextParams.dFilterMedianBuffer, []);
                const count = Number(getValue(contextParams.dFilterMedianCount, 0));
                return [ size, buffer, count ];
            };
            const pidValues = () => {
                const p = getValue(contextParams.prevP, 0);
                const i = getValue(contextParams.prevI, 0);
                const d = getValue(contextParams.prevD, 0);
                return [Number(p), Number(i), Number(d)];
            };
            let clearCache = strToBool(getValue(contextParams.clearCache, 'false', false));
            if (clearCache) {
                for (let e of Object.values(contextParams)) nodeContext.set(e, null);
                msg.payload = 0;
                send([msg, null, null]);
                done();
                return;
            }
            let entry = Number(getValue('payload', 0));
            let target = Number(getValue(contextParams.target, 0));
            let initialEntry = Number(getValue(contextParams.initialEntry, -1));
            if (initialEntry === -1) {
                nodeContext.set(contextParams.initialEntry, entry);
                initialEntry = entry;
            }
            let prevError = Number(getValue(contextParams.prevError, 0));
            let prevOutput = Number(getValue(contextParams.prevOutput, 0));
            let force = strToBool(getValue(contextParams.force, 'false'));
            let [ kp, ki, kd ] = pidCoeffs();
            let lastCalculation = Number(getValue(contextParams.lastCalculationTime, 0));
            let direction = getValue(contextParams.direction, 'normal');
            let [ clipOutput, outputMinLimit, outputMaxLimit ] = clipOutputConfig();
            let [ clipI, iMinLimit, iMaxLimit ] = clipIConfig();
            let clipIASOutput = strToBool(getValue(contextParams.clipIAsOutput, 'true'));
            let pCalculationMode = getValue(contextParams.pCalculationMode, 'error');
            let iCalcualtionMode = getValue(contextParams.iCalculationMode, 'normal');
            let iWindowSize = Number(getValue(contextParams.iWindowSize, 0));
            let iWindow = getValue(contextParams.iWindow, []);
            let filters = filtersConfig();
            let [ kalmanFilterEMea, kalmanFilterEEst, kalmanFilterQ, kalmanFilterLastEstimate ] = kalmanFilterConfig();
            let [ emaFilterLength, emaFilterPrev, emaFilterStep ] = emaFilterConfig();
            let [ smaFilterLength, smaFilterAccumulated ] = smaFilterConfig();
            let [ medianFilterSize, medianFilterBuffer, medianFilterCount ] = medianFilterConfig();
            let [ prevP, prevI, prevD ] = pidValues();
            let fallbackValue = Number(getValue('fallbackValue', 0));
            let sendState = strToBool(getValue(contextParams.sendState, 'false'));
            let sendConfig = strToBool(getValue(contextParams.sendConfig, 'false', false));

            function applyKalmanFilter(entry, eMea, eEst, q, lastEstimate, eEstCfgName, lastEstimateCfgName) {
                const [ output, outputEEst ] = kalmanFilter(entry, eMea, eEst, q, lastEstimate, safeDivisionUnit);
                nodeContext.set(eEstCfgName, outputEEst);
                nodeContext.set(lastEstimateCfgName, lastEstimate);
                for (let e of nodeContext.keys()) context[e] = nodeContext.get(e);
                return output;
            }

            function applyEMAFilter(entry, length, prev, step, prevCfgName, stepCfgName) {
                const [ output, outputPrev, outputStep ] = emaFilter(entry, length, prev, step, safeDivisionUnit);
                nodeContext.set(prevCfgName, outputPrev);
                nodeContext.set(stepCfgName, outputStep);
                for (let e of nodeContext.keys()) context[e] = nodeContext.get(e);
                return output;
            }

            function applySMAFilter(entry, length, accumulated, accumulatedCfgName) {
                const [ output, outputAccumulated ] = smaFilter(entry, length, accumulated, safeDivisionUnit);
                nodeContext.set(accumulatedCfgName, outputAccumulated);
                for (let e of nodeContext.keys()) context[e] = nodeContext.get(e);
                return output;
            }

            function applyMedianFilter(entry, size, buffer, count, bufferCfgName, countCfgName) {
                const [ output, outputBuffer, outputCount ] = medianFilter(entry, size, buffer, count);
                nodeContext.set(bufferCfgName, outputBuffer);
                nodeContext.set(countCfgName, outputCount);
                for (let e of nodeContext.keys()) context[e] = nodeContext.get(e);
                return output;
            }

            let output = 0;
            let dFiltersHistory = [];
            const now = Date.now();
            let t = Number(getValue('t', 0));
            let dynamicT = t === 0;
            t = t === 0 ? now - lastCalculation : t;
            let tS = t / 1000;
            let error = 0;
            if (now - lastCalculation >= t || (force || dynamicT)) {
                let p = pCalculationMode === 'error' ? onErrorProportional(target, entry) : onRateProportional(entry, initialEntry);
                error = target - entry;
                nodeContext.set(contextParams.prevError, error);
                nodeContext.set(contextParams.prevP, p);
                let i = 0;
                if (ki !== 0) {
                    if (iCalcualtionMode === 'normal') {
                        i = onErrorIntegral(prevI, error, tS);
                    } else {
                        i = iWindow.length >= 1 ? onWindowIntegral(iWindow, iWindowSize) : onErrorIntegral(prevI, error, tS);
                        iWindow.push(error);
                        iWindow = iWindow.slice(-iWindowSize);
                        nodeContext.set(contextParams.iWindow, iWindow);
                    }
                    i = clipI ? clip(i, iMinLimit, iMaxLimit) : i;
                    i = clipIASOutput ? clip (i, 0, outputMaxLimit) : i;
                }
                nodeContext.set(contextParams.prevI, i);
                let d = (error - prevError) / (tS + safeDivisionUnit);
                dFiltersHistory.push(d);
                for (let e of filters) {
                    switch (e) {
                        case 'kalman':
                            d = applyKalmanFilter(d, kalmanFilterEMea, kalmanFilterEEst, kalmanFilterQ, kalmanFilterLastEstimate, contextParams.dFilterKalmanEEst, contextParams.dFilterKalmanLastEstimate);
                            dFiltersHistory.push(d);
                            break;
                        case 'ema':
                            d = applyEMAFilter(d, emaFilterLength, emaFilterPrev, emaFilterStep, contextParams.dFilterEMAPrev, contextParams.dFilterEMAStep);
                            dFiltersHistory.push(d);
                            break;
                        case 'sma':
                            d = applySMAFilter(d, smaFilterLength, smaFilterAccumulated, contextParams.dFilterSMAAccumulated);
                            dFiltersHistory.push(d);
                            break;
                        case 'median':
                            d = applyMedianFilter(d, medianFilterSize, medianFilterBuffer, medianFilterCount, contextParams.dFilterMedianBuffer, contextParams.dFilterMedianCount);
                            dFiltersHistory.push(d);
                            break;
                        default:
                            dFiltersHistory.push(d);
                            break;
                    }
                }
                nodeContext.set(contextParams.prevD, d);
                kp = iCalcualtionMode === 'rate' ? -kp : kp;
                if ( direction === 'inverse') {
                    kp = kp > 0 ? -kp : Math.abs(kp);
                    ki = -ki;
                    kd = -kd;
                }
                output = (p * kp) + (i * ki) + (d * kd);
                output = clipOutput ? clip(output, outputMinLimit, outputMaxLimit) : output;
                nodeContext.set(contextParams.lastCalculationTime, now);
                nodeContext.set(contextParams.prevOutput, output);
            } else {
                output = prevOutput;
            }
            let errorDetected = Number.isNaN(output) ? true : false
            output = errorDetected ? fallbackValue : output
            msg.payload = output;
            let state = null;
            let cfg = null
            if (sendConfig) {
                cfg = {
                    root: {
                        target: target,
                        kp: Math.abs(kp),
                        ki: Math.abs(ki),
                        kd: Math.abs(kd),
                        t: t,
                        direction: direction,
                        clipOutput: clipOutput,
                        outputMinLimit: outputMinLimit,
                        outputMaxLimit: outputMaxLimit,
                        clipI: clipI,
                        iMinLimit: iMinLimit,
                        iMaxLimit: iMaxLimit,
                        clipIASOutput: clipIASOutput,
                        iCalcualtionMode: iCalcualtionMode,
                        iWindowSize: iWindowSize,
                        dFilter1: filters[0],
                        dFilter2: filters[1],
                        dFilter3: filters[2],
                        dFilter4: filters[3],
                        dFilterKalmanEMea: kalmanFilterEMea,
                        dFilterKalmanEEst: kalmanFilterEEst,
                        dFilterKalmanQ: kalmanFilterQ,
                        dFilterEMALength: emaFilterLength,
                        dFilterSMALength: smaFilterLength,
                        dFilterMedianSize: medianFilterSize,
                        fallbackValue: fallbackValue,
                        sendState: sendState
                    }
                };
            }
            if (sendState) {
                state = {
                    root: {
                        target: target,
                        entry: entry,
                        initialEntry: initialEntry,
                        errorDetected: errorDetected,
                        t: t,
                        tS: tS,
                        lastCalculationTime: lastCalculation,
                        force: force,
                        iWindow: iWindow,
                        dFiltersHistory: dFiltersHistory,
                        dFilterKalmanEEst: kalmanFilterEEst,
                        dFilterKalmanLastEstimate: kalmanFilterLastEstimate,
                        dFilterEMAPrev: emaFilterPrev,
                        dFilterEMAStep: emaFilterStep,
                        dFilterSMAAccumulated: smaFilterAccumulated,
                        dFilterMedianBuffer: medianFilterBuffer,
                        dFilterMedianCount: medianFilterCount,
                        p: prevP,
                        i: prevI,
                        d: prevD,
                        multP: prevP * kp,
                        multI: prevI * ki,
                        multD: prevD * kd,
                        output: output,
                        prevOutput: prevOutput,
                        error: error,
                        prevError: prevError
                    }
                };
            }
            send([msg, state, cfg]);
            done();
        });
        this.on('close', () => { for (let e of Object.values(context)) this.context().set(e, null); });
    }
    RED.nodes.registerType('pid-regulator', pidRegulator);
}
