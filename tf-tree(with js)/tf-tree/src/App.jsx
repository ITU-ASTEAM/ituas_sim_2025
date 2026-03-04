import { useState, useCallback } from "react";

const DEFAULT_TF = `world
  map
    odom
      base_link
        base_footprint
        imu_link
        laser_link
        camera_link
          camera_rgb_frame
            camera_rgb_optical_frame
          camera_depth_frame
            camera_depth_optical_frame
        left_wheel
        right_wheel`;

function parseTFInput(raw) {
  const lines = raw.split("\n").filter((l) => l.trim());
  const isTreeFormat = lines.some((l) => /[├└│─]/.test(l));
  if (isTreeFormat) return parseTreeFormat(lines);
  return parseIndentFormat(lines);
}

function parseTreeFormat(lines) {
  const stack = [], roots = [];
  for (const line of lines) {
    const indent = line.search(/[^\s│├└─ ]/);
    const name = line.replace(/^[\s│├└─]+/, "").trim();
    if (!name) continue;
    const node = { name, children: [] };
    while (stack.length && stack[stack.length-1].indent >= indent) stack.pop();
    if (!stack.length) roots.push(node);
    else stack[stack.length-1].node.children.push(node);
    stack.push({ indent, node });
  }
  return roots;
}

function parseIndentFormat(lines) {
  const stack = [], roots = [];
  for (const line of lines) {
    if (!line.trim()) continue;
    const indent = line.match(/^(\s*)/)[1].length;
    const name = line.trim();
    const node = { name, children: [] };
    while (stack.length && stack[stack.length-1].indent >= indent) stack.pop();
    if (!stack.length) roots.push(node);
    else stack[stack.length-1].node.children.push(node);
    stack.push({ indent, node });
  }
  return roots;
}

function countNodes(node) {
  return 1 + node.children.reduce((s, c) => s + countNodes(c), 0);
}
function maxDepth(node, d=0) {
  if (!node.children.length) return d;
  return Math.max(...node.children.map(c => maxDepth(c, d+1)));
}

function TreeNode({ node, prefix="", isLast=true, depth=0 }) {
  const [collapsed, setCollapsed] = useState(false);
  const connector = isLast ? "└── " : "├── ";
  const childPrefix = prefix + (isLast ? "    " : "│   ");
  const colors = ["#7ee8a2","#56c8e8","#e8a256","#c878e8","#e87878","#78e8d8"];
  const nameColor = colors[Math.min(depth, colors.length-1)];
  const hasChildren = node.children.length > 0;

  return (
    <div style={{ fontFamily:"'JetBrains Mono',monospace" }}>
      <div
        style={{ display:"flex", alignItems:"center", whiteSpace:"pre", lineHeight:"1.8", cursor: hasChildren?"pointer":"default" }}
        onClick={() => hasChildren && setCollapsed(c => !c)}
      >
        <span style={{ color:"#3d4a5c" }}>{prefix}</span>
        <span style={{ color:"#3d4a5c" }}>{connector}</span>
        {hasChildren && (
          <span style={{ color:"#556070", fontSize:"10px", marginRight:"5px", display:"inline-block", transform: collapsed?"rotate(-90deg)":"rotate(0deg)", transition:"transform 0.15s" }}>▾</span>
        )}
        <span style={{ color:nameColor, fontWeight:depth===0?"700":"400", fontSize:depth===0?"14px":"13px", textShadow:depth===0?`0 0 10px ${nameColor}55`:"none" }}>
          {node.name}
        </span>
        {hasChildren && collapsed && (
          <span style={{ color:"#3d4a5c", fontSize:"11px", marginLeft:"8px" }}>[+{countNodes(node)-1}]</span>
        )}
      </div>
      {!collapsed && node.children.map((child,i) => (
        <TreeNode key={child.name+i} node={child} prefix={childPrefix} isLast={i===node.children.length-1} depth={depth+1} />
      ))}
    </div>
  );
}

export default function TFTreeViewer() {
  const [input, setInput] = useState(DEFAULT_TF);
  const [trees, setTrees] = useState(() => parseTFInput(DEFAULT_TF));
  const [showInput, setShowInput] = useState(false);
  const [copied, setCopied] = useState(false);

  const handleParse = useCallback(() => {
    setTrees(parseTFInput(input));
    setShowInput(false);
  }, [input]);

  const totalNodes = trees.reduce((s,t) => s+countNodes(t), 0);
  const totalDepth = trees.length ? Math.max(...trees.map(t => maxDepth(t))) : 0;

  const renderText = (node, prefix="", isLast=true) => {
    const c = isLast?"└── ":"├── ";
    const cp = prefix+(isLast?"    ":"│   ");
    let o = prefix+c+node.name+"\n";
    node.children.forEach((ch,i) => { o += renderText(ch, cp, i===node.children.length-1); });
    return o;
  };

  const copyTree = () => {
    const text = trees.map(t => t.name+"\n"+t.children.map((c,i)=>renderText(c,"",i===t.children.length-1)).join("")).join("\n");
    navigator.clipboard.writeText(text);
    setCopied(true);
    setTimeout(()=>setCopied(false), 1500);
  };

  return (
    <div style={{ minHeight:"100vh", background:"#090e16", display:"flex", flexDirection:"column", alignItems:"center", padding:"32px 16px" }}>
      <div style={{ width:"100%", maxWidth:"860px", marginBottom:"16px" }}>
        <div style={{ display:"flex", alignItems:"center", gap:"10px", marginBottom:"8px" }}>
          {["#ff5f57","#febc2e","#28c840"].map(c=><div key={c} style={{ width:12,height:12,borderRadius:"50%",background:c }}/>)}
          <span style={{ color:"#3d4a5c", fontSize:"12px", fontFamily:"monospace" }}>ros2 tf tree — terminal</span>
        </div>
        <div style={{ background:"#0f1520", border:"1px solid #1a2234", borderRadius:"8px", padding:"8px 16px", fontFamily:"monospace", fontSize:"13px" }}>
          <span style={{ color:"#3d4a5c" }}>$ </span>
          <span style={{ color:"#58a6ff" }}>ros2 run tf2_tools view_frames</span>
        </div>
      </div>

      <div style={{ width:"100%", maxWidth:"860px", background:"#0f1520", border:"1px solid #1a2234", borderRadius:"10px", overflow:"hidden", boxShadow:"0 24px 64px rgba(0,0,0,0.6)" }}>
        <div style={{ background:"#131c2b", borderBottom:"1px solid #1a2234", padding:"10px 16px", display:"flex", justifyContent:"space-between", alignItems:"center" }}>
          <span style={{ color:"#3d4a5c", fontSize:"12px", fontFamily:"monospace" }}>
            <span style={{ color:"#7ee8a2" }}>{totalNodes} frame</span>{" · "}
            <span style={{ color:"#56c8e8" }}>derinlik: {totalDepth}</span>
          </span>
          <div style={{ display:"flex", gap:"8px" }}>
            <button onClick={copyTree} style={{ background:copied?"#1a3a2a":"#1a2234", border:"1px solid #243044", borderRadius:"5px", color:copied?"#7ee8a2":"#556070", fontSize:"11px", padding:"3px 10px", cursor:"pointer", fontFamily:"monospace" }}>
              {copied?"✓ kopyalandı":"kopyala"}
            </button>
            <button onClick={()=>setShowInput(v=>!v)} style={{ background:showInput?"#1a2a44":"#1a2234", border:"1px solid #243044", borderRadius:"5px", color:showInput?"#58a6ff":"#556070", fontSize:"11px", padding:"3px 10px", cursor:"pointer", fontFamily:"monospace" }}>
              {showInput?"kapat":"düzenle"}
            </button>
          </div>
        </div>

        {showInput && (
          <div style={{ borderBottom:"1px solid #1a2234", padding:"14px 16px", background:"#090e16" }}>
            <div style={{ color:"#556070", fontSize:"11px", marginBottom:"8px", fontFamily:"monospace" }}>
              TF ağacınızı yapıştırın (girintili liste veya ├─ tree formatı):
            </div>
            <textarea value={input} onChange={e=>setInput(e.target.value)} rows={10}
              style={{ width:"100%", background:"#0f1520", border:"1px solid #243044", borderRadius:"6px", color:"#c9d1d9", fontFamily:"'JetBrains Mono',monospace", fontSize:"12px", padding:"10px", resize:"vertical", outline:"none", boxSizing:"border-box" }}
            />
            <button onClick={handleParse} style={{ marginTop:"8px", background:"#1f6feb", border:"none", borderRadius:"6px", color:"#fff", fontSize:"12px", padding:"6px 18px", cursor:"pointer", fontFamily:"monospace" }}>
              Görselleştir →
            </button>
          </div>
        )}

        <div style={{ padding:"20px 24px 28px", overflowX:"auto" }}>
          <div style={{ color:"#7ee8a2", fontSize:"13px", marginBottom:"8px", fontFamily:"monospace", fontWeight:"700" }}>.</div>
          {trees.length===0
            ? <div style={{ color:"#3d4a5c", fontSize:"13px", fontFamily:"monospace" }}>Ağaç verisi bulunamadı.</div>
            : trees.map((tree,i) => <TreeNode key={i} node={tree} prefix="" isLast={true} depth={0} />)
          }
          <div style={{ color:"#2a3548", fontSize:"11px", marginTop:"16px", fontFamily:"monospace" }}>
            {totalNodes} frame, {totalDepth} levels deep
          </div>
        </div>
      </div>

      <div style={{ marginTop:"20px", display:"flex", gap:"16px", flexWrap:"wrap", justifyContent:"center" }}>
        {[["#7ee8a2","kök"],["#56c8e8","seviye 1"],["#e8a256","seviye 2"],["#c878e8","seviye 3"],["#e87878","seviye 4"],["#78e8d8","seviye 5+"]].map(([color,label]) => (
          <div key={label} style={{ display:"flex", alignItems:"center", gap:"5px" }}>
            <div style={{ width:7,height:7,borderRadius:"50%",background:color }}/>
            <span style={{ color:"#3d4a5c", fontSize:"11px", fontFamily:"monospace" }}>{label}</span>
          </div>
        ))}
      </div>
      <style>{`
        @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;700&display=swap');
        ::-webkit-scrollbar{width:5px;height:5px}
        ::-webkit-scrollbar-track{background:#090e16}
        ::-webkit-scrollbar-thumb{background:#1a2234;border-radius:3px}
      `}</style>
    </div>
  );
}