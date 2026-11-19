#!/usr/bin/env node
/*
 * Gravity Lab — headless self-test runner.
 *
 * Extracts the physics engine (<script id="physics-engine">) from gravity_lab.html
 * and runs the same automated self-tests that the browser runs on page load.
 * The physics engine has no DOM dependencies, so it runs unchanged in Node.js.
 *
 * Usage:   node tests/run_tests.js
 * Exit code is 0 if every test passes, 1 otherwise.
 */
'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const htmlPath = path.join(__dirname, '..', 'gravity_lab.html');
const html = fs.readFileSync(htmlPath, 'utf8');

const open = '<script id="physics-engine">';
const start = html.indexOf(open);
if (start < 0) { console.error('physics-engine script not found in gravity_lab.html'); process.exit(1); }
const end = html.indexOf('</script>', start);
const code = html.slice(start + open.length, end);

const sandbox = { console, performance: require('perf_hooks').performance, module: { exports: {} } };
vm.createContext(sandbox);
vm.runInContext(code, sandbox, { filename: 'physics-engine.js' });
const Phys = sandbox.module.exports;

const { results, npass } = Phys.runSelfTests(msg => console.log(String(msg).replace('%c', '')));
console.log('');
console.log(npass === results.length ? 'ALL TESTS PASSED' : (results.length - npass) + ' TEST(S) FAILED');
process.exit(npass === results.length ? 0 : 1);
