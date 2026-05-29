"""
药物知识库示例数据 (drugs.json)
"""
import json

SAMPLE_DRUGS = [
    {
        "drug_id": "DRG-001",
        "name": "阿莫西林胶囊",
        "generic_name": "Amoxicillin",
        "category": "抗生素/青霉素类",
        "indications": ["上呼吸道感染", "急性扁桃体炎", "中耳炎", "支气管炎"],
        "contraindications": ["青霉素过敏者禁用", "肾功能不全者慎用"],
        "dosage": {"adult": "0.5g，每日3次", "child": "20-40mg/kg/日，分3次"},
        "shelf_location": "A-03-12",
    },
    {
        "drug_id": "DRG-002",
        "name": "布洛芬缓释胶囊",
        "generic_name": "Ibuprofen",
        "category": "解热镇痛抗炎药",
        "indications": ["头痛", "发热", "关节痛", "牙痛", "痛经"],
        "contraindications": ["胃溃疡患者禁用", "哮喘患者慎用", "孕妇慎用"],
        "dosage": {"adult": "0.3g，每日2次", "child": "5-10mg/kg，每日3次"},
        "shelf_location": "B-05-03",
    },
    {
        "drug_id": "DRG-003",
        "name": "对乙酰氨基酚片",
        "generic_name": "Paracetamol",
        "category": "解热镇痛药",
        "indications": ["发热", "头痛", "肌肉酸痛", "感冒"],
        "contraindications": ["肝肾功能不全者禁用", "每日不超过2g"],
        "dosage": {"adult": "0.5g，每日3-4次", "child": "10-15mg/kg，每日3-4次"},
        "shelf_location": "B-02-08",
    },
    {
        "drug_id": "DRG-004",
        "name": "头孢克肟胶囊",
        "generic_name": "Cefixime",
        "category": "抗生素/头孢类",
        "indications": ["呼吸道感染", "泌尿道感染", "中耳炎"],
        "contraindications": ["头孢过敏者禁用", "青霉素过敏者慎用(交叉过敏)"],
        "dosage": {"adult": "0.1g，每日2次", "child": "3-6mg/kg，每日2次"},
        "shelf_location": "A-01-15",
    },
    {
        "drug_id": "DRG-005",
        "name": "复方甘草片",
        "generic_name": "Compound Liquorice",
        "category": "镇咳祛痰药",
        "indications": ["咳嗽", "咳痰", "支气管炎"],
        "contraindications": ["高血压患者慎用", "长期使用可能依赖"],
        "dosage": {"adult": "2-3片，每日3次", "child": "遵医嘱"},
        "shelf_location": "C-02-20",
    },
]

if __name__ == "__main__":
    with open("drugs.json", "w", encoding="utf-8") as f:
        json.dump(SAMPLE_DRUGS, f, ensure_ascii=False, indent=2)
    print(f"Generated drugs.json with {len(SAMPLE_DRUGS)} drugs")
