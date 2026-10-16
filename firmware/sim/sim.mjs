// sim.mjs -- build firmware/sim/sim.cpp (with hotaru_motion.ino inside) to
// WebAssembly with LLVM clang, and run it.
//
//   node firmware/sim/sim.mjs            exit code 0 = every test passed
//
// Needs LLVM (clang + wasm-ld); no C library, no Arduino toolchain.
import { execFileSync } from "node:child_process";
import { readFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const clang = process.env.CLANG ||
  (process.platform === "win32" ? "C:\\Program Files\\LLVM\\bin\\clang++.exe" : "clang++");
const out = join(tmpdir(), "hotaru_sim.wasm");
execFileSync(clang, [
  "--target=wasm32", "-std=c++17", "-O2", "-nostdlib", "-fno-exceptions", "-fno-rtti",
  "-Wall", "-Wextra", "-Wshadow", "-Wno-missing-field-initializers",
  "-Wl,--no-entry", "-Wl,--allow-undefined", "-I", here, join(here, "sim.cpp"), "-o", out,
], { stdio: "inherit" });

const f = (x, n = 1) => x.toFixed(n).padStart(6);
const pass = (ok) => (ok ? "PASS" : "FAIL");
const ROUTES = ["fold-back -80 -> +80", "fold-back -60 -> +60",
                "reach-back -90 -> +90", "rest -90 -> +90"];
const say = (code, i, a, b, c, d) => {
  if (code === 1) {
    if (i === 0) console.log("1. pan an arm across the button pod");
    console.log(`   ${ROUTES[i].padEnd(22)} straight line ${f(a)} mm | firmware: ` +
                `${b} leg(s), closest ${f(c)} mm  ${pass(d)}`);
  } else if (code === 2) {
    console.log(`2. button / demo cycle: ${i} moves, closest ${f(a)} mm, ` +
                `${c} not reached  ${pass(b)}`);
  } else if (code === 3) {
    console.log(`3. new command halfway through a pan (pan at ${a.toFixed(0)} deg, ` +
                `${b.toFixed(0)} deg/s), retargeted to fold-back at +60: closest ${f(c)} mm  ${pass(d)}`);
  } else if (code === 4) {
    console.log(`4. ${i} random targets: ${a} refused by the pose guard, ${b} with no clear ` +
                `route, ${c} went round the pod`);
  } else if (code === 5) {
    console.log(`   ${a} frames run, closest ${f(b)} mm, frames inside a keep-out: ${c}  ${pass(d)}`);
  }
};
const { instance } = await WebAssembly.instantiate(readFileSync(out), {
  env: { sinf: Math.sin, cosf: Math.cos, say },
});
const fails = instance.exports.run();
console.log(fails ? `\nFAILED (${fails})` : "\nall passed");
process.exit(fails ? 1 : 0);
