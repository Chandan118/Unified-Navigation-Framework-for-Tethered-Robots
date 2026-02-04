import base64
import os
import json

files_to_upload = [
    'setup.sh', 'docker-compose.yaml', 'Dockerfile', 'ros_entrypoint.sh',
    'run_full_sim.sh', 'result_analyzer.py', 'test_environment.py'
]

js_files = []
for f in files_to_upload:
    if os.path.exists(f):
        with open(f, 'rb') as fd:
            content = base64.b64encode(fd.read()).decode('utf-8')
            js_files.append({ 'path': f, 'content': content })

js_content = f'''
(async () => {{
  const token = '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';
  const owner = 'Chandan118';
  const repo = 'Unified-Navigation-Framework-for-Tethered-Robots';
  
  const files = {json.dumps(js_files)};

  for (const file of files) {{
    const url = `https://api.github.com/repos/${{owner}}/${{repo}}/contents/${{file.path}}`;
    try {{
        let sha = null;
        const getRes = await fetch(url, {{ headers: {{ 'Authorization': `token ${{token}}` }} }});
        if (getRes.ok) {{
            const data = await getRes.json();
            sha = data.sha;
        }}
        
        const response = await fetch(url, {{
          method: 'PUT',
          headers: {{
            'Authorization': `token ${{token}}`,
            'Content-Type': 'application/json'
          }},
          body: JSON.stringify({{
            message: `Add ${{file.path}}`,
            content: file.content,
            sha: sha
          }})
        }});
        console.log(`Uploaded ${{file.path}}: `, await response.json());
    }} catch (e) {{
        console.error(`Failed to upload ${{file.path}}:`, e);
    }}
  }}
}})();
'''

with open('root_files_upload.js', 'w') as f:
    f.write(js_content)
