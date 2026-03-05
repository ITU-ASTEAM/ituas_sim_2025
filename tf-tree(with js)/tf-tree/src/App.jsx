import { useState, useEffect, useCallback } from "react";

const API = "http://localhost:8000";

function countNodes(node) {
  return 1 + node.children.reduce((s, c) => s + countNodes(c), 0);
}
function maxDepth(node, d = 0) {
  if (!node.children.length) return d;
  return Math.max(...node.children.map((c) => maxDepth(c, d + 1)));
}

function TreeNode({ node, prefix = "", isLast = true, depth = 0 }) {
  const [collapsed, setCollapsed] = useState(false);
  const connector = isLast ? "└── " : "├── ";
  const childPrefix = prefix + (isLast ? "    " : "│   ");
  const colors = ["#7ee8a2", "#56c8e8", "#e8a256", "#c878e8", "#e87878", "#78e8d8"];
  const nameColor = colors[Math.min(depth, colors.length - 1)];
  const hasChildren = node.children.length > 0;

  return (
    <div style={{ fontFamily: "'JetBrains Mono',monospace" }}>
      <div
        style={{ display: "flex", alignItems: "center", whiteSpace: "pre", lineHeight: "1.8", cursor: hasChildren ? "pointer" : "default" }}
        onClick={() => hasChildren && setCollapsed((c) => !c)}
      >
        <span style={{ color: "#3d4a5c" }}>{prefix}</span>
        <span style={{ color: "#3d4a5c" }}>{connector}</span>
        {hasChildren && (
          <span style={{ color: "#556070", fontSize: "10px", marginRight: "5px", display: "inline-block", transform: collapsed ? "rotate(-90deg)" : "rotate(0deg)", transition: "transform 0.15s" }}>▾</span>
        )}
        <span style={{ color: nameColor, fontWeight: depth === 0 ? "700" : "400", fontSize: depth === 0 ? "14px" : "13px", textShadow: depth === 0 ? `0 0 10px ${nameColor}55` : "none" }}>
          {node.name}
        </span>
        {hasChildren && collapsed && (
          <span style={{ color: "#3d4a5c", fontSize: "11px", marginLeft: "8px" }}>[+{countNodes(node) - 1}]</span>
        )}
      </div>
      {!collapsed &&
        node.children.map((child, i) => (
          <TreeNode key={child.name + i} node={child} prefix={childPrefix} isLast={i === node.children.length - 1} depth={depth + 1} />
        ))}
    </div>
  );
}

export default function TFTreeViewer() {
  const [trees, setTrees] = useState([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [simTime, setSimTime] = useState(false);
  const [lastFetch, setLastFetch] = useState(null);
  const [copied, setCopied] = useState(false);

  const fetchTree = useCallback(async (sim) => {
    setLoading(true);
    setError(null);
    try {
      const res = await fetch(`${API}/tf-tree?sim_time=${sim ?? simTime}`);
      const data = await res.json();
      if (data.success) {
        setTrees(data.tree);
        setLastFetch(new Date().toLocaleTimeString("tr-TR"));
      } else {
        setError(data.error);
        setTrees([]);
      }
    } catch {
      setError("Backend'e bağlanılamadı. server.py çalışıyor mu? (port 8000)");
      setTrees([]);
    } finally {
      setLoading(false);
    }
  }, [simTime]);

  useEffect(() => { fetchTree(false); }, []);

  const totalNodes = trees.reduce((s, t) => s + countNodes(t), 0);
  const totalDepth = trees.length ? Math.max(...trees.map((t) => maxDepth(t))) : 0;

  const renderText = (node, prefix = "", isLast = true) => {
    const c = isLast ? "└── " : "├── ";
    const cp = prefix + (isLast ? "    " : "│   ");
    let o = prefix + c + node.name + "\n";
    node.children.forEach((ch, i) => { o += renderText(ch, cp, i === node.children.length - 1); });
    return o;
  };

  const copyTree = () => {
    const text = trees.map((t) => t.name + "\n" + t.children.map((c, i) => renderText(c, "", i === t.children.length - 1)).join("")).join("\n");
    navigator.clipboard.writeText(text);
    setCopied(true);
    setTimeout(() => setCopied(false), 1500);
  };

  return (
    <div style={{ minHeight: "100vh", background: "#090e16", display: "flex", flexDirection: "column", alignItems: "center", padding: "32px 16px" }}>
      <div style={{ width: "100%", maxWidth: "860px", marginBottom: "16px" }}>
        <div style={{ display: "flex", alignItems: "center", gap: "10px", marginBottom: "8px" }}>
          {["#ff5f57", "#febc2e", "#28c840"].map((c) => (
            <div key={c} style={{ width: 12, height: 12, borderRadius: "50%", background: c }} />
          ))}
          <span style={{ color: "#3d4a5c", fontSize: "12px", fontFamily: "monospace" }}>ros2 tf tree — terminal</span>
        </div>
        <div style={{ background: "#0f1520", border: "1px solid #1a2234", borderRadius: "8px", padding: "8px 16px", fontFamily: "monospace", fontSize: "13px" }}>
          <span style={{ color: "#3d4a5c" }}>$ </span>
          <span style={{ color: "#58a6ff" }}>ros2 run tf2_tools view_frames{simTime ? " --ros-args -p use_sim_time:=true" : ""}</span>
        </div>
      </div>

      <div style={{ width: "100%", maxWidth: "860px", background: "#0f1520", border: "1px solid #1a2234", borderRadius: "10px", overflow: "hidden", boxShadow: "0 24px 64px rgba(0,0,0,0.6)" }}>
        <div style={{ background: "#131c2b", borderBottom: "1px solid #1a2234", padding: "10px 16px", display: "flex", justifyContent: "space-between", alignItems: "center", flexWrap: "wrap", gap: "8px" }}>
          <span style={{ color: "#3d4a5c", fontSize: "12px", fontFamily: "monospace" }}>
            {loading
              ? <span style={{ color: "#556070" }}>⟳ ROS2'den çekiliyor...</span>
              : trees.length > 0
                ? <><span style={{ color: "#7ee8a2" }}>{totalNodes} frame</span>{" · "}<span style={{ color: "#56c8e8" }}>derinlik: {totalDepth}</span>{lastFetch && <span style={{ color: "#2a3a4a" }}>{" · "}{lastFetch}</span>}</>
                : <span style={{ color: "#556070" }}>veri yok</span>
            }
          </span>
          <div style={{ display: "flex", gap: "8px", alignItems: "center" }}>
            <label style={{ display: "flex", alignItems: "center", gap: "5px", cursor: "pointer", color: simTime ? "#e8a256" : "#3d4a5c", fontSize: "11px", fontFamily: "monospace" }}>
              <div
                onClick={() => { const v = !simTime; setSimTime(v); fetchTree(v); }}
                style={{ width: 28, height: 15, borderRadius: 8, background: simTime ? "#3a2a0a" : "#1a2234", border: `1px solid ${simTime ? "#e8a256" : "#243044"}`, position: "relative", cursor: "pointer", transition: "all 0.2s" }}
              >
                <div style={{ position: "absolute", top: 2, left: simTime ? 13 : 2, width: 9, height: 9, borderRadius: "50%", background: simTime ? "#e8a256" : "#3d4a5c", transition: "left 0.2s" }} />
              </div>
              sim_time
            </label>
            {trees.length > 0 && (
              <button onClick={copyTree} style={{ background: copied ? "#1a3a2a" : "#1a2234", border: "1px solid #243044", borderRadius: "5px", color: copied ? "#7ee8a2" : "#556070", fontSize: "11px", padding: "3px 10px", cursor: "pointer", fontFamily: "monospace" }}>
                {copied ? "✓ kopyalandı" : "kopyala"}
              </button>
            )}
            <button onClick={() => fetchTree()} disabled={loading} style={{ background: loading ? "#1a2234" : "#1a2a44", border: "1px solid #243044", borderRadius: "5px", color: loading ? "#2a3a4a" : "#58a6ff", fontSize: "11px", padding: "3px 12px", cursor: loading ? "not-allowed" : "pointer", fontFamily: "monospace" }}>
              {loading ? "⟳ bekleniyor..." : "⟳ yenile"}
            </button>
          </div>
        </div>

        <div style={{ padding: "20px 24px 28px", overflowX: "auto", minHeight: "120px" }}>
          {loading ? (
            <div style={{ color: "#3d4a5c", fontSize: "13px", fontFamily: "monospace", lineHeight: "2" }}>
              <div style={{ color: "#556070" }}>$ ros2 run tf2_tools view_frames</div>
              <div style={{ animation: "pulse 1.5s infinite" }}>Listening to tf data for 5.0 seconds...</div>
            </div>
          ) : error ? (
            <div style={{ fontFamily: "monospace" }}>
              <div style={{ color: "#e87878", fontSize: "12px", marginBottom: "12px" }}>✗ {error}</div>
              <div style={{ color: "#2a3a4a", fontSize: "11px", lineHeight: "1.8" }}>
                Kontrol listesi:<br />
                1. ROS2 kurulu mu? → <span style={{ color: "#3d4a5c" }}>ros2 --version</span><br />
                2. server.py çalışıyor mu? → <span style={{ color: "#3d4a5c" }}>python3 server.py</span><br />
                3. Port 8000 açık mı? → <span style={{ color: "#3d4a5c" }}>curl http://localhost:8000/health</span>
              </div>
            </div>
          ) : trees.length === 0 ? (
            <div style={{ color: "#3d4a5c", fontSize: "13px", fontFamily: "monospace" }}>TF frame bulunamadı. ROS2 sistemi çalışıyor mu?</div>
          ) : (
            <>
              <div style={{ color: "#7ee8a2", fontSize: "13px", marginBottom: "8px", fontFamily: "monospace", fontWeight: "700" }}>.</div>
              {trees.map((tree, i) => <TreeNode key={i} node={tree} prefix="" isLast={true} depth={0} />)}
            </>
          )}
        </div>
      </div>

      <div style={{ marginTop: "20px", display: "flex", gap: "16px", flexWrap: "wrap", justifyContent: "center" }}>
        {[["#7ee8a2", "kök"], ["#56c8e8", "seviye 1"], ["#e8a256", "seviye 2"], ["#c878e8", "seviye 3"], ["#e87878", "seviye 4"], ["#78e8d8", "seviye 5+"]].map(([color, label]) => (
          <div key={label} style={{ display: "flex", alignItems: "center", gap: "5px" }}>
            <div style={{ width: 7, height: 7, borderRadius: "50%", background: color }} />
            <span style={{ color: "#3d4a5c", fontSize: "11px", fontFamily: "monospace" }}>{label}</span>
          </div>
        ))}
      </div>

      <style>{`
        @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;700&display=swap');
        @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.4} }
        ::-webkit-scrollbar{width:5px;height:5px}
        ::-webkit-scrollbar-track{background:#090e16}
        ::-webkit-scrollbar-thumb{background:#1a2234;border-radius:3px}
      `}</style>
    </div>
  );
}
