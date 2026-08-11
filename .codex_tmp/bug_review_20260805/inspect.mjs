import fs from "node:fs/promises";
import { Workbook } from "@oai/artifact-tool";

const path = "C:/Users/watar/OneDrive/デスクトップ/Twelite_pc_tool/converted/謎のバグ.csv";
const workbook = await Workbook.fromCSV(
  await fs.readFile(path, "utf8"),
  { sheetName: "Log" },
);
const result = await workbook.inspect({
  kind: "workbook,sheet,table",
  maxChars: 4000,
  tableMaxRows: 5,
  tableMaxCols: 20,
  tableMaxCellChars: 60,
});
console.log(result.ndjson);
