// Parse the embedded page and exercise its state-rendering path without hardware.
import fs from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';
const text=fs.readFileSync(new URL('../esp32/.firmware/web_dashboard.cpp',import.meta.url),'utf8');
const scripts=[...text.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(x=>x[1]);
assert.equal(scripts.length,1);
new vm.Script(scripts[0]);
const elements=new Map();
for(const m of text.matchAll(/\bid="([^"]+)"/g)) {
  assert(!elements.has(m[1]),`duplicate DOM id ${m[1]}`);
  elements.set(m[1],{value:'',textContent:'',innerHTML:'',style:{},classList:{add(){},remove(){},toggle(){}},setAttribute(){},querySelector(){return null;},addEventListener(){}});
}
const context={console,Date,Math,JSON,Number,String,Array,parseInt,parseFloat,isNaN,
  location:{hostname:'localhost',host:'localhost',protocol:'http:'},
  document:{getElementById:id=>elements.get(id)||null,activeElement:null,querySelectorAll:()=>[],addEventListener(){}},
  localStorage:{getItem(){return null;},setItem(){}},
  setTimeout(){},clearTimeout(){},setInterval(){},clearInterval(){},
  fetch:()=>Promise.resolve({ok:true,json:()=>Promise.resolve({})}),
  WebSocket:class {static OPEN=1; readyState=1;send(){}},
};
context.window=context;
vm.createContext(context);
vm.runInContext(scripts[0],context);
context.upd({op_mode:1,hw_version:3,fsd_protocol_mode:14,hw3_offset_mode:2,
  hw3_manual_offset:12,hw3_custom_pct:[30,20,10,10],private399_limit_seen:true,
  private399_limit_kph:60,speed_offset:50,research_target_kph:90,research_smooth_kph:90,
  research_cap_kph:90,mock_can0:2,mock_can1:3,chip_temp_c:45,chip_temp_max_c:50});
assert.equal(String(elements.get('offsetMode').value),'2');
assert.equal(String(elements.get('cp0').value),'30');
assert.match(elements.get('researchValues').textContent,/can0=2，can1=3/);
assert.match(elements.get('btnMode').textContent,/模拟/);
assert(!text.includes('MARK TAP'));
console.log('dashboard: JavaScript syntax, unique IDs, state render and research controls passed');
