import{L as e,T as t,Y as n,_ as r,b as i,g as a,gt as o,mt as s,x as c}from"./modules/shiki-BTitm6Yo.js";import{nt as l,rt as u}from"./index-AC2flFN5.js";import{t as d}from"./slidev/default-Bc_zM1C0.js";import{t as f}from"./slidev/CodeBlockWrapper-CmC5pjDB.js";import{t as p}from"./slidev/Mermaid-DzoK0Hrx.js";var m={class:`grid grid-cols-2 gap-4`},h={class:`bg-gray-50 p-4 rounded-lg`},g={__name:`pharmacy_bot_答辩PPT.md__slidev_12`,setup(g){let{$slidev:_,$nav:v,$clicksContext:y,$clicks:b,$page:x,$renderContext:S,$frontmatter:C}=u();return y.setup(),(u,g)=>{let _=p,v=f;return e(),r(d,o(t(s(l)(s(C),11))),{default:n(()=>[g[5]||=a(`h1`,null,`08 / AI 智能诊断系统 (代码实现)`,-1),a(`div`,m,[a(`div`,null,[g[0]||=a(`h3`,null,[i(`🧠 症状→推荐管线 (`),a(`code`,null,`ai_agent/`),i(`)`)],-1),c(_,{"code-lz":`GYGw9g7gxgFghgJwC4AIAyAlAUC3KCCA2gESDjroG1Og4aaA05sQLooC0jAfCgEIkCiAdkgJZIAnii4APJAjhQBYHvRx52TVigDCJNTARgAtnAAi7ADwAjBAHoWgRBVA84mABi0BEvgrzqVbAyQAKO3QAdUQBDzQAIEs0sWAApAahUMfABxFCg5JABTCQBKFzwDd1ESNDQAWQIvAEkwqwMUlL8AZWqAawsARQgU+TpFXC5cgDESAClagHkAORRAZb9AHPMKlkBjyMAE80Bk+MAvxXogA`}),g[1]||=a(`h3`,null,`📁 核心代码文件`,-1),g[2]||=a(`table`,null,[a(`thead`,null,[a(`tr`,null,[a(`th`,null,`文件`),a(`th`,null,`功能`)])]),a(`tbody`,null,[a(`tr`,null,[a(`td`,null,[a(`code`,null,`symptom_agent.py`)]),a(`td`,null,`症状→RAG→LLM 管线`)]),a(`tr`,null,[a(`td`,null,[a(`code`,null,`drug_kb.py`)]),a(`td`,null,`ChromaDB 向量知识库`)]),a(`tr`,null,[a(`td`,null,[a(`code`,null,`llm_client.py`)]),a(`td`,null,`OpenAI兼容API封装`)]),a(`tr`,null,[a(`td`,null,[a(`code`,null,`drugs_sample.py`)]),a(`td`,null,`5种药品样本数据`)]),a(`tr`,null,[a(`td`,null,[a(`code`,null,`config.py`)]),a(`td`,null,`Provider/API配置`)])])],-1)]),a(`div`,h,[g[4]||=a(`h3`,null,`🔍 RAG 检索 + LLM 推理流程`,-1),c(v,{title:``,ranges:[]},{default:n(()=>[...g[3]||=[a(`pre`,{class:`shiki shiki-themes vitesse-dark vitesse-light slidev-code`,style:{"--shiki-dark":`#dbd7caee`,"--shiki-light":`#393a34`,"--shiki-dark-bg":`#121212`,"--shiki-light-bg":`#ffffff`}},[a(`code`,{class:`language-text`},[a(`span`,{class:`line`},[a(`span`,null,`用户输入: "我头痛、发烧两天了"`)]),i(`
`),a(`span`,{class:`line`},[a(`span`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`↓ Entity Extraction`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`症状: ["头痛", "发烧"]`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`持续时间: "两天"`)]),i(`
`),a(`span`,{class:`line`},[a(`span`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`↓ ChromaDB 向量检索`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`匹配药物: 布洛芬 (相似度 0.92)`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`          阿莫西林 (相似度 0.65)`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`          氯雷他定 (相似度 0.31)`)]),i(`
`),a(`span`,{class:`line`},[a(`span`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`↓ Prompt 构造 (含RAG + JSON schema)`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`→ LLM API 调用`)]),i(`
`),a(`span`,{class:`line`},[a(`span`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`↓ JSON 输出`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`{`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`  "analysis": "可能为上呼吸道感染...",`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`  "recommendations": [`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`    {"drug": "布洛芬缓释胶囊", "reason": "镇痛退热"}`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`  ],`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`  "precautions": "肝肾功能不全者慎用",`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`  "urgent_warning": false`)]),i(`
`),a(`span`,{class:`line`},[a(`span`,null,`}`)])])],-1)]]),_:1})])])]),_:1},16)}}};export{g as default};