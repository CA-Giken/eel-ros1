// vite.config.js
import { defineConfig } from 'vite'
import { resolve, join } from 'path'
import { readdirSync, writeFileSync, statSync } from 'fs'

function scanDirectory(dir) {
  let jsFiles = [];

  const items = readdirSync(dir);

  for (const item of items) {
    const fullPath = join(dir, item);
    const stat = statSync(fullPath);

    if (stat.isDirectory()) {
      // ディレクトリの場合は再帰的に探索
      jsFiles = [...jsFiles, ...scanDirectory(fullPath)];
    } else if (stat.isFile() && item.endsWith('.js')) {
      // モックファイルの場合はスキップ
      if (item.endsWith('.mock.js')) {
        continue;
      }
      // 相対パスに変換（jsDir からの相対パス）
      const relativePath = fullPath.replace(resolve(__dirname, 'templates'), '.');
      jsFiles.push(relativePath);
    }
  }

  return jsFiles;
}

// エントリーポイントの生成
const jsDir = resolve(__dirname, 'templates');
const jsFiles = scanDirectory(jsDir);

// importの順序を制御したい場合の設定
const priorityFiles = [
  './const.js',
  './utils.js',

  // 他の優先ファイル
];

// ignoreFiles に含まれるファイルは除外
const ignoreFiles = [
  "./ros.js"
]


// 優先ファイルを先頭に、それ以外のファイルをソートして追加
const sortedFiles = [
  ...priorityFiles.filter(file => jsFiles.includes(file)),
  ...jsFiles.filter(file => !priorityFiles.includes(file))
    .filter(file => !ignoreFiles.includes(file))
    .sort()
];

// import文の生成
const imports = sortedFiles
  .filter(file => !file.endsWith('auto-entry.js'))
  .map(file => `import '${file}';`)
  .join('\n');

// デバッグ用のログ出力
console.log('Found JS files:');
console.log(sortedFiles);

// エントリーファイルの生成
const entryFile = resolve(__dirname, 'templates/auto-entry.js');
writeFileSync(entryFile, imports);

export default defineConfig({
  build: {
    lib: {
      entry: entryFile,
      name: 'EelRos1Bundle',
      fileName: 'bundle',
      formats: ['umd']
    },
    rollupOptions: {
      output: {
        extend: true
      }
    },
    outDir: 'dist/web',
    emptyOutDir: false,
    sourcemap: true,
    minify: 'terser'
  }
})