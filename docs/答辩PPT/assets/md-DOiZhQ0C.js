import{L as e,T as t,Y as n,_ as r,b as i,g as a,gt as o,mt as s,x as c}from"./modules/shiki-BTitm6Yo.js";import{nt as l,rt as u}from"./index-AC2flFN5.js";import{t as d}from"./slidev/default-Bc_zM1C0.js";import{t as f}from"./slidev/CodeBlockWrapper-CmC5pjDB.js";import{t as p}from"./slidev/Mermaid-DzoK0Hrx.js";var m={class:`grid grid-cols-2 gap-4`},h={class:`bg-green-50 p-4 rounded-lg`},g={__name:`pharmacy_bot_答辩PPT.md__slidev_13`,setup(g){let{$slidev:_,$nav:v,$clicksContext:y,$clicks:b,$page:x,$renderContext:S,$frontmatter:C}=u();return y.setup(),(u,g)=>{let _=p,v=f;return e(),r(d,o(t(s(l)(s(C),12))),{default:n(()=>[g[5]||=a(`h1`,null,`09 / 视觉识别系统 (代码实现)`,-1),a(`div`,m,[a(`div`,null,[g[0]||=a(`h3`,null,[i(`👁️ 三级视觉管道 (`),a(`code`,null,`ocr_barcode/`),i(`)`)],-1),c(_,{"code-lz":`GYGw9g7gxgFghgJwC4AIAqARAUC3KCCA2gEQDCcAtgKYJwoBit1xAuigLTsB8KAQiQE0A8gBkhANwAcAHgBGCAPRdAd26BwC0ADFoGlbVjjy8O3FKRIAFOABNzIKkNIAlOYq6Bw00Dq2oDHowNRKOvHwM8MEgAHAE8AL1lERyVAQ3NAQA9ADgtAGKyfPFJ/FABREgwEAFcAcxQMKiQqKCQwBGiuQEg5QCwEwGUEwFjwwAgVVNwMDMzdXEyM+hIAZi7AWXlAf81ABnlAQuiawCTCFDshAGUUAFkwcSoASSRWIA===`}),g[1]||=a(`h3`,null,`📁 核心代码文件`,-1),g[2]||=a(`table`,null,[a(`thead`,null,[a(`tr`,null,[a(`th`,null,`文件`),a(`th`,null,`功能`),a(`th`,null,`状态`)])]),a(`tbody`,null,[a(`tr`,null,[a(`td`,null,[a(`code`,null,`barcode_scanner.py`)]),a(`td`,null,`多ROI+多帧条码扫描`),a(`td`,null,`✅ 可用`)]),a(`tr`,null,[a(`td`,null,[a(`code`,null,`ocr_scanner.py`)]),a(`td`,null,`PaddleOCR文字识别`),a(`td`,null,`✅ 可用`)]),a(`tr`,null,[a(`td`,null,[a(`code`,null,`drug_detector.py`)]),a(`td`,null,`三通道融合判优`),a(`td`,null,`✅ 可用`)]),a(`tr`,null,[a(`td`,null,[a(`code`,null,`tests/test_all.py`)]),a(`td`,null,`回归测试`),a(`td`,null,`✅ 5/5通过`)])])],-1)]),a(`div`,h,[g[4]||=a(`h3`,null,`🔬 三通道融合判优逻辑`,-1),c(v,{title:``,ranges:[]},{default:n(()=>[...g[3]||=[a(`pre`,{class:`shiki shiki-themes vitesse-dark vitesse-light slidev-code`,style:{"--shiki-dark":`#dbd7caee`,"--shiki-light":`#393a34`,"--shiki-dark-bg":`#121212`,"--shiki-light-bg":`#ffffff`}},[a(`code`,{class:`language-text`},[a(`span`,{class:`line`},[a(`span`,null,`输入: 药盒ROI图像`)]),i(`
`),a(`span`,{class:`line`},[a(`span`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`┌─────────────────────┐`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`│ Channel 1: 条码     │ ← pyzbar: EAN-13解码`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`│   置信度: 0.95      │`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`├─────────────────────┤`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`│ Channel 2: 药名     │ ← PaddleOCR: "阿莫西林胶囊"`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`│   置信度: 0.87      │`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`├─────────────────────┤`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`│ Channel 3: 颜色     │ ← HSV: 蓝色包装`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`│   匹配度: 0.92      │`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`├─────────────────────┤`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`│ 融合评分: 0.91     │ ← 加权平均`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`│ 核验结果: ✅ 通过   │`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`└─────────────────────┘`)]),i(`
`),a(`span`,{class:`line`},[a(`span`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`判决规则:`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`- 任一通道 >0.95 → 直接通过`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`- 融合评分 >0.70 → 通过`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`- 融合评分 0.5~0.7 → 重拍1次`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`- 融合评分 <0.5 → 告警人工介入`)])])],-1)]]),_:1})])])]),_:1},16)}}};export{g as default};