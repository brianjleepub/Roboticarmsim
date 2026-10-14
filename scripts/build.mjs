// Bundles src/ into a single self-contained HTML file (no runtime CDN dependencies).
//   dist/index.html     full document, for GitHub Pages or any static host
//   dist/artifact.html  body fragment for the claude.ai Artifact host (it adds the <html>/<head>
//                       skeleton); loads the bundle from dist/app.js published alongside it
import * as esbuild from 'esbuild';
import { readFile, writeFile, mkdir } from 'node:fs/promises';

const watch = process.argv.includes('--watch');

async function emit() {
  const out = await esbuild.build({
    entryPoints: ['src/main.js'],
    bundle: true,
    minify: true,
    format: 'iife',
    target: ['es2020', 'safari16'],
    write: false,
    legalComments: 'none',
  });
  const js = out.outputFiles[0].text.replace(/<\/script/gi, '<\\/script');
  const page = await readFile('src/page.html', 'utf8');
  const [head, body] = page.split('<!--BODY-->');
  const script = `<script>${js}</script>`;
  const bodyHtml = body.replace('<!--SCRIPT-->', () => script);
  await mkdir('dist', { recursive: true });
  const full = `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
${head.trim()}
</head>
<body>
${bodyHtml.trim()}
</body>
</html>
`;
  await writeFile('dist/index.html', full);
  await writeFile('dist/app.js', out.outputFiles[0].text);
  const artifactBody = body.replace('<!--SCRIPT-->', '<script src="app.js"></script>');
  await writeFile('dist/artifact.html', `${head.trim()}\n${artifactBody.trim()}\n`);
  console.log(`built dist/index.html (${(full.length / 1024).toFixed(0)} KB)`);
}

await emit();
if (watch) {
  const { watch: fsWatch } = await import('node:fs');
  let t;
  fsWatch('src', { recursive: true }, () => { clearTimeout(t); t = setTimeout(() => emit().catch(console.error), 100); });
  console.log('watching src/ …');
}
