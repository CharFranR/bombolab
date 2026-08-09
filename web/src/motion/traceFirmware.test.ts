import { describe, it, expect } from 'vitest';
import { parseTelemetryLine, firmwareTraceCsv, firmwareTraceStats, type FirmwareSample } from './traceFirmware';

describe('parseTelemetryLine', () => {
  it('parsea una T-line válida', () => {
    const s = parseTelemetryLine('T 50000 1500 1500 1500 1500 1500 1472');
    expect(s).toEqual({ tUs: 50000, joints: [1500, 1500, 1500, 1500, 1500, 1472] });
  });

  it('rechaza líneas que no son T', () => {
    expect(parseTelemetryLine('ACK 24')).toBeNull();
    expect(parseTelemetryLine('T 100')).toBeNull();
  });
});

describe('firmwareTraceCsv', () => {
  it('emite header + filas', () => {
    const samples: FirmwareSample[] = [
      { tUs: 0, joints: [1500, 1500, 1500, 1500, 1500, 1472] },
      { tUs: 50000, joints: [1600, 1500, 1500, 1500, 1500, 1472] },
    ];
    const csv = firmwareTraceCsv(samples);
    expect(csv.split('\n')[0]).toBe('t_us,j1_us,j2_us,j3_us,j4_us,j5_us,g_us');
    expect(csv.trim().split('\n')).toHaveLength(3);
    expect(csv).toContain('50000,1600,1500,1500,1500,1500,1472');
  });
});

describe('firmwareTraceStats', () => {
  it('calcula count y duración', () => {
    const samples: FirmwareSample[] = [
      { tUs: 0, joints: [] },
      { tUs: 100000, joints: [] },
      { tUs: 200000, joints: [] },
    ];
    expect(firmwareTraceStats(samples)).toEqual({ count: 3, durationUs: 200000 });
  });

  it('vacío → ceros', () => {
    expect(firmwareTraceStats([])).toEqual({ count: 0, durationUs: 0 });
  });
});
