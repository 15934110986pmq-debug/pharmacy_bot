"""
药物知识库 — ChromaDB 本地向量检索
"""
import json
import logging
from typing import List

import chromadb
from chromadb.utils import embedding_functions

logger = logging.getLogger(__name__)


class DrugKnowledgeBase:
    """药物知识库 — 本地向量检索"""

    def __init__(self, db_path="./drug_kb_store"):
        self.client = chromadb.PersistentClient(path=db_path)
        self.ef = embedding_functions.DefaultEmbeddingFunction()
        self.collection = self.client.get_or_create_collection(
            name="drugs", embedding_function=self.ef
        )
        logger.info(f"DrugKnowledgeBase initialized: path={db_path}")

    def load_drugs(self, drugs_json_path: str):
        """
        从JSON加载药物数据，构建向量索引

        JSON格式:
        [
          {
            "drug_id": "DRG-001",
            "name": "阿莫西林胶囊",
            "generic_name": "Amoxicillin",
            "category": "抗生素/青霉素类",
            "indications": ["上呼吸道感染", "急性扁桃体炎"],
            "contraindications": ["青霉素过敏者禁用"],
            "dosage": {"adult": "0.5g，每日3次", "child": "20-40mg/kg/日"},
            "shelf_location": "A-03-12"
          }
        ]
        """
        with open(drugs_json_path, encoding="utf-8") as f:
            drugs = json.load(f)

        ids, documents, metadatas = [], [], []
        for drug in drugs:
            # 构建检索文本: 名称 + 适应症 + 禁忌（用于向量匹配）
            doc_parts = [drug["name"], drug.get("generic_name", "")]
            doc_parts.extend(drug.get("indications", []))
            doc_parts.extend(drug.get("contraindications", []))

            ids.append(drug["drug_id"])
            documents.append(" ".join(doc_parts))
            metadatas.append(
                {
                    "name": drug["name"],
                    "category": drug.get("category", ""),
                    "indications": json.dumps(drug.get("indications", []), ensure_ascii=False),
                    "contraindications": json.dumps(
                        drug.get("contraindications", []), ensure_ascii=False
                    ),
                    "dosage": json.dumps(drug.get("dosage", {}), ensure_ascii=False),
                    "shelf_location": drug.get("shelf_location", ""),
                }
            )

        # 如果集合已有数据，先清空
        existing = self.collection.get()
        if existing["ids"]:
            self.collection.delete(ids=existing["ids"])

        self.collection.add(ids=ids, documents=documents, metadatas=metadatas)
        logger.info(f"Loaded {len(drugs)} drugs into knowledge base from {drugs_json_path}")

    def retrieve(self, query: str, k: int = 5) -> List[dict]:
        """
        症状 → Top-K 匹配药物

        Args:
            query: 症状描述文本
            k: 返回数量

        Returns:
            [{drug_id, drug_name, indications, contraindications, dosage, shelf_location}, ...]
        """
        results = self.collection.query(query_texts=[query], n_results=k)

        if not results["ids"] or not results["ids"][0]:
            logger.warning(f"No drugs matched for query: {query}")
            return []

        drugs = []
        for mid, meta, dist in zip(
            results["ids"][0], results["metadatas"][0], results["distances"][0]
        ):
            # 距离转相似度 (cosine distance → similarity)
            similarity = 1 - dist if dist else 1.0
            drugs.append(
                {
                    "drug_id": mid,
                    "drug_name": meta["name"],
                    "category": meta.get("category", ""),
                    "indications": json.loads(meta.get("indications", "[]")),
                    "contraindications": json.loads(meta.get("contraindications", "[]")),
                    "dosage": json.loads(meta.get("dosage", "{}")),
                    "shelf_location": meta.get("shelf_location", ""),
                    "similarity": round(similarity, 3),
                }
            )

        logger.info(f"Retrieved {len(drugs)} drugs for query (top similarity={drugs[0]['similarity']})")
        return drugs

    def get_by_id(self, drug_id: str) -> dict | None:
        """按ID查询单个药物"""
        result = self.collection.get(ids=[drug_id])
        if not result["ids"]:
            return None
        meta = result["metadatas"][0]
        return {
            "drug_id": drug_id,
            "drug_name": meta["name"],
            "category": meta.get("category", ""),
            "indications": json.loads(meta.get("indications", "[]")),
            "contraindications": json.loads(meta.get("contraindications", "[]")),
            "dosage": json.loads(meta.get("dosage", "{}")),
            "shelf_location": meta.get("shelf_location", ""),
        }
