export interface FirmwareSample {
  tUs: number;
  joints: number[];
}

export function parseTelemetryLine(line: string): FirmwareSample | null {
  const m = /^T (\d+) ((\d+ ){5}\d+)$/.exec(line);
  if (!m) return null;
  return { tUs: Number(m[1]), joints: m[2].split(' ').map(Number) };
}

export function firmwareTraceCsv(samples: FirmwareSample[]): string {
  const header = 't_us,j1_us,j2_us,j3_us,j4_us,j5_us,g_us';
  const rows = samples.map((s) => `${s.tUs},${s.joints.join(',')}`);
  return [header, ...rows].join('\n') + '\n';
}

export function firmwareTraceStats(samples: FirmwareSample[]): { count: number; durationUs: number } {
  if (samples.length === 0) return { count: 0, durationUs: 0 };
  const first = samples[0].tUs;
  const last = samples[samples.length - 1].tUs;
  return { count: samples.length, durationUs: Math.max(0, last - first) };
}
