// Firefox WebDriver BiDi로 로컬 검수 화면을 headless 검증한다. 추가 npm 의존성 없음.
import {spawn} from 'node:child_process';
import {mkdtemp, writeFile, rm} from 'node:fs/promises';
import {dirname, resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
import assert from 'node:assert/strict';

const [htmlArg, outputArg] = process.argv.slice(2);
if (!htmlArg || !outputArg) throw Error('사용법: node review_browser_check.mjs <HTML> <결과 JSON>');
const html=resolve(htmlArg), output=resolve(outputArg);
// Snap Firefox도 접근 가능한 입력 폴더에 전용 임시 profile을 만든다.
const profile=await mkdtemp(dirname(html)+'/browser-profile-');
const proc=spawn('/usr/bin/firefox',['--headless','--no-remote','--profile',profile,'--remote-debugging-port','0'],{stdio:['ignore','pipe','pipe']});
let ws, nextId=0;
const pending=new Map(), checks=[];
function call(method,params={}){
 return new Promise((resolve,reject)=>{
  const id=++nextId, timer=setTimeout(()=>{pending.delete(id);reject(Error('BiDi 응답 시간 초과: '+method));},20000);
  pending.set(id,{resolve:value=>{clearTimeout(timer);resolve(value);},reject:error=>{clearTimeout(timer);reject(error);}});
  ws.send(JSON.stringify({id,method,params}));
 });
}
try{
 const endpoint=await new Promise((resolve,reject)=>{
  let log='';const timer=setTimeout(()=>reject(Error('Firefox 시작 시간 초과: '+log)),15000);
  proc.stderr.on('data',chunk=>{log+=chunk;const match=log.match(/WebDriver BiDi listening on (ws:\/\/[^\s]+)/);if(match){clearTimeout(timer);resolve(match[1]);}});
  proc.stdout.resume();proc.once('exit',code=>{clearTimeout(timer);reject(Error('Firefox 종료: '+code+' '+log));});
 });
 ws=new WebSocket(endpoint+'/session');
 await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
 ws.onmessage=event=>{const message=JSON.parse(event.data);const job=pending.get(message.id);if(!job)return;pending.delete(message.id);message.type==='error'?job.reject(Error(JSON.stringify(message))):job.resolve(message.result);};
 const session=await call('session.new',{capabilities:{}});
 const {context}=await call('browsingContext.create',{type:'tab'});
 await call('browsingContext.navigate',{context,url:pathToFileURL(html).href,wait:'complete'});
 async function evaluate(expression){
  const reply=await call('script.evaluate',{expression:`(async()=>JSON.stringify(await (${expression})))()`,target:{context},awaitPromise:true});
  if(reply.type==='exception')throw Error(JSON.stringify(reply));
  return JSON.parse(reply.result.value);
 }
 const waitReady=()=>evaluate(`new Promise((resolve,reject)=>{let n=0;const timer=setInterval(()=>{if(ready){clearInterval(timer);resolve(true);}else if(++n>200){clearInterval(timer);reject(Error('영상 로딩 시간 초과'));}},20);})`);
 await waitReady();
 const initial=await evaluate('({frames:data.frames.length,width:canvas.width,height:canvas.height})');
 assert(initial.frames>1);checks.push('image_loaded');
 const hasContext=await evaluate('Object.keys(current().review_context || {}).length>0');
 if(hasContext){
  assert.equal(await evaluate(`$('context').textContent===JSON.stringify(current().review_context,null,2) && $('context').tagName==='PRE' && !$('context').isContentEditable`),true);
  checks.push('readonly_context');
 }
 const result=await evaluate(`(()=>{
  $('add').click();$('door').value=String(current().doors.length-1);
  const rect=canvas.getBoundingClientRect();
  // 캔버스가 화면 안에 들어오도록 스크롤한 후 실제 pointer 입력을 사용한다.
  canvas.scrollIntoView();
  const r=canvas.getBoundingClientRect();
  return {x:Math.round(r.left+20*r.width/canvas.width),y:Math.round(r.top+25*r.height/canvas.height),
          x2:Math.round(r.left+120*r.width/canvas.width),y2:Math.round(r.top+160*r.height/canvas.height)};
 })()`);
 await call('input.performActions',{context,actions:[{type:'pointer',id:'mouse',parameters:{pointerType:'mouse'},actions:[
  {type:'pointerMove',x:result.x,y:result.y},{type:'pointerDown',button:0},
  {type:'pointerMove',x:result.x2,y:result.y2,duration:100},{type:'pointerUp',button:0}]}]});
 const box=await evaluate(`current().doors.at(-1).leaf_bbox`);
 assert(box && box[2]>box[0] && box[3]>box[1]);checks.push('pointer_bbox');
 assert.equal(await evaluate(`(()=>{$('reviewed').checked=true;$('reviewed').dispatchEvent(new Event('change'));return current().reviewed;})()`),true);
 checks.push('explicit_review');
 assert.equal(await evaluate(`(()=>{$('labels').value='invalid';$('reviewed').checked=false;$('reviewed').dispatchEvent(new Event('change'));const cleared=!current().reviewed;renderLabels();return cleared;})()`),true);
 checks.push('uncheck_with_invalid_json');
 const parentId=await evaluate('current().id');
 assert.equal(await evaluate(`(()=>{show(1);try{saveEditor();return false;}catch(e){return true;}})()`),true);
 checks.push('loading_guard');
 await waitReady();assert.notEqual(await evaluate('current().id'),parentId);
 const restored=await evaluate(`(async()=>{
  const p=patch();p.frames[0].doors.at(-1).state='open';
  const transfer=new DataTransfer();transfer.items.add(new File([JSON.stringify(p)],'patch.json',{type:'application/json'}));
  $('restore').files=transfer.files;const task=$('restore').onchange({target:$('restore')});
  let guarded=false;try{saveEditor();}catch(e){guarded=true;}
  const locked=restoring && $('labels').disabled && $('restore').disabled && guarded;
  await task;return {state:data.frames[0].doors.at(-1).state,locked};
 })()`);
 assert.equal(restored.state,'open');assert.equal(restored.locked,true);checks.push('restore_patch','restore_edit_guard');await waitReady();
 const exported=await evaluate('patch()');
 assert.equal(exported.frames.length,initial.frames);
 assert.equal(exported.frames[0].reviewed,false);
 checks.push('patch_export');
 // 출처와 UI 결과만 기록한다. 시험 편집을 사람 검수 자료로 저장하지 않는다.
 await writeFile(output,JSON.stringify({status:'passed',browser:session.capabilities.browserVersion,
  frames:initial.frames,checks,limitations:['headless UI 동작 검사이며 사람 라벨 검수·정답 평가가 아님']},null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({status:'passed',frames:initial.frames,checks}));
 await call('session.end');
}finally{
 ws?.close();
 for(const job of pending.values())job.reject(Error('검사 종료'));
 if(proc.exitCode===null && proc.signalCode===null){
  const done=new Promise(resolve=>proc.once('exit',resolve));proc.kill('SIGTERM');await done;
 }
 await rm(profile,{recursive:true,force:true});
}
