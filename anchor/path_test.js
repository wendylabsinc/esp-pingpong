// The page's path of sweeps (follow() in mesh.html), run on made-up lists of sounds:  node path_test.js mesh.html
const fs = require('fs'), assert = require('assert');
const h = fs.readFileSync(process.argv[2] || 'mesh.html', 'utf8');
const kindLine = h.split('\n').find(l => l.startsWith('const kind = q =>'));
const body = h.slice(h.indexOf('const PATH_MAX'), h.indexOf('const seen = {};'));
const make = new Function(kindLine + '\n' + body + '\nreturn { chain, follow };');
const e = { major_m: 0.2, minor_m: 0.07, angle_rad: 0 };
// a sound heard at time t, as the Anchor lists it at time `now`
const at = (now, list) => list.map(([event, t, x, y, note = 'sweep heard', far = false]) => ({ event, age_s: now - t, x, y, note, far, ellipse: e }));
const keys = c => c.nodes.map(n => n.key).join('');

{ // every sweep is a point, in the order heard, however near or far from the one before
  const { chain, follow } = make();
  const all = [['A', 0, 0, 0], ['B', 0.4, 0.02, 0], ['C', 0.8, 30, 5], ['D', 1.2, 0.02, 0]];
  follow(chain, at(1.3, all), 1.3, 5);
  assert.strictEqual(keys(chain), 'ABCD');
  follow(chain, at(1.8, all), 1.8, 5);                                    // the same list again: nothing twice
  assert.strictEqual(keys(chain), 'ABCD');
  follow(chain, at(2.5, [['E', 2.4, 1, 1], ...all].reverse()), 2.5, 5);   // whatever order the list is in
  assert.strictEqual(keys(chain), 'ABCDE');
}
{ // the path ends after `quiet` seconds without a sweep, and the next sweep begins a new one
  const { chain, follow } = make();
  follow(chain, at(1, [['A', 0, 0, 0], ['B', 0.5, 1, 0]]), 1, 5);
  follow(chain, at(5.4, [['A', 0, 0, 0], ['B', 0.5, 1, 0]]), 5.4, 5);
  assert.strictEqual(keys(chain), 'AB');                                  // 4.9 s of silence: still the path
  follow(chain, at(5.6, [['A', 0, 0, 0], ['B', 0.5, 1, 0]]), 5.6, 5);
  assert.strictEqual(keys(chain), '');                                    // 5.1 s: ended
  assert.strictEqual(chain.old.nodes.length, 2);
  assert(Math.abs(chain.old.at - 5.5) < 1e-9);
  follow(chain, at(9, [['A', 0, 0, 0], ['B', 0.5, 1, 0], ['C', 8.9, 2, 0]]), 9, 5);
  assert.strictEqual(keys(chain), 'C');                                   // A and B do not come back
  follow(chain, at(9.5, [['B', 0.5, 1, 0], ['C', 8.9, 2, 0], ['D', 9.3, 3, 0]]), 9.5, 5);
  assert.strictEqual(keys(chain), 'CD');
}
{ // a longer quiet time keeps one path where a shorter one makes two
  const list = [['A', 0, 0, 0], ['B', 8, 1, 0]];
  const long = make(); long.follow(long.chain, at(8.1, list), 8.1, 10);
  assert.strictEqual(keys(long.chain), 'AB');
  const short = make(); short.follow(short.chain, at(8.1, list), 8.1, 5);
  assert.strictEqual(keys(short.chain), 'B');
}
{ // sweeps that keep coming from one spot are one point, kept up to date, and keep the path alive
  const { chain, follow } = make();
  follow(chain, at(1, [['A', 0, 0, 0], ['R', 0.9, 1, 0, 'sweep heard']]), 1, 5);
  for (let t = 1.4; t < 9; t += 0.4) follow(chain, at(t + 0.1, [['A', 0, 0, 0], ['R', t, 1.01, 0.02, 'sweep heard, 9 times']]), t + 0.1, 5);
  assert.strictEqual(keys(chain), 'AR');
  assert.strictEqual(chain.nodes[1].x, 1.01);
  assert.strictEqual(chain.nodes[1].note, 'sweep heard, 9 times');
}
{ // a sweep with a direction only has no point, but it is a sweep: the path lives on
  const { chain, follow } = make();
  follow(chain, at(0.1, [['A', 0, 0, 0]]), 0.1, 5);
  follow(chain, at(4.1, [['A', 0, 0, 0], ['F', 4, 0, 0, 'sweep heard', true]]), 4.1, 5);
  for (let now = 4.6; now < 8; now += 0.5) follow(chain, at(now, [['A', 0, 0, 0], ['F', 4, 0, 0, 'sweep heard', true]]), now, 5);
  follow(chain, at(8.1, [['A', 0, 0, 0], ['F', 4, 0, 0, 'sweep heard', true], ['B', 8, 1, 0]]), 8.1, 5);
  assert.strictEqual(keys(chain), 'AB');
}
{ // other kinds of sound are never on the path, and do not keep it alive
  const { chain, follow } = make();
  follow(chain, at(1, [['A', 0, 0, 0], ['x', 0.5, 1, 0, 'faint sound'], ['y', 0.7, 1, 0, 'buzzer of 4824'], ['z', 0.8, 1, 0, null]]), 1, 5);
  assert.strictEqual(keys(chain), 'A');
  follow(chain, at(6, [['A', 0, 0, 0], ['x', 5.5, 1, 0, 'faint sound']]), 6, 5);
  assert.strictEqual(keys(chain), '');
}
{ // a page opened on a list of old sweeps shows no path; one opened mid-walk shows the recent stretch
  const old = make(); old.follow(old.chain, at(100, [['A', 0, 0, 0], ['B', 0.4, 1, 0], ['C', 50, 2, 0]]), 100, 5);
  assert.strictEqual(keys(old.chain), '');
  const mid = make(); mid.follow(mid.chain, at(100, [['A', 0, 0, 0], ['B', 97, 1, 0], ['C', 99, 2, 0]]), 100, 5);
  assert.strictEqual(keys(mid.chain), 'BC');
}
{ // a sweep whose place comes in late goes where it belongs in time
  const { chain, follow } = make();
  follow(chain, at(1, [['A', 0, 0, 0], ['C', 0.8, 2, 0]]), 1, 5);
  follow(chain, at(1.5, [['A', 0, 0, 0], ['B', 0.4, 1, 0], ['C', 0.8, 2, 0]]), 1.5, 5);
  assert.strictEqual(keys(chain), 'ABC');
}
console.log('path ok');
