import { runParserSelfTests } from '../src/lib/gcodeCipra.ts';

const { ok, failures } = runParserSelfTests();
for (const failure of failures) {
  console.error(`parity failure: ${failure}`);
}
if (!ok) {
  console.error(`gcodeCipra parser self-test failed: ${failures.length} failure(s)`);
  process.exit(1);
}
console.log('gcodeCipra parser self-test: ok');
