#!/usr/bin/env python3
"""
Demo server script for sending POST request to /TransferServerOntoPlatform
向192.168.1.42:7001发送POST请求到/TransferServerOntoPlatform端点
"""

import requests
import json
import logging
from typing import Dict, Any, Optional

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class DemoServerClient:
    def __init__(self, host: str = "192.168.1.42", port: int = 7001):
        """
        初始化Demo服务器客户端
        
        Args:
            host: 服务器主机地址
            port: 服务器端口
        """
        self.base_url = f"http://{host}:{port}"
        self.session = requests.Session()
        
    def transfer_server_onto_platform(self, data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        发送POST请求到/TransferServerOntoPlatform端点
        
        Args:
            data: 要发送的数据，如果为None则发送空的JSON对象
            
        Returns:
            服务器响应的JSON数据
            
        Raises:
            requests.RequestException: 当请求失败时抛出异常
        """
        url = f"{self.base_url}/TransferServerOntoPlatform"
        
        # 如果没有提供数据，使用空的字典
        if data is None:
            data = {}
            
        headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        }
        
        try:
            logger.info(f"发送POST请求到: {url}")
            logger.info(f"请求数据: {json.dumps(data, indent=2)}")
            
            response = self.session.post(
                url,
                json=data,
                headers=headers,
                timeout=30
            )
            
            # 检查响应状态码
            response.raise_for_status()
            
            # 尝试解析JSON响应
            try:
                response_data = response.json()
                logger.info(f"响应成功: {json.dumps(response_data, indent=2)}")
                return response_data
            except json.JSONDecodeError:
                # 如果响应不是JSON格式，返回文本内容
                logger.info(f"响应成功 (非JSON): {response.text}")
                return {"response_text": response.text, "status_code": response.status_code}
                
        except requests.exceptions.ConnectionError as e:
            logger.error(f"连接错误: 无法连接到 {url}")
            logger.error(f"错误详情: {str(e)}")
            raise
        except requests.exceptions.Timeout as e:
            logger.error(f"请求超时: {url}")
            logger.error(f"错误详情: {str(e)}")
            raise
        except requests.exceptions.HTTPError as e:
            logger.error(f"HTTP错误: {e.response.status_code} - {e.response.reason}")
            logger.error(f"响应内容: {e.response.text}")
            raise
        except Exception as e:
            logger.error(f"未知错误: {str(e)}")
            raise

def main():
    """
    主函数 - 演示如何使用DemoServerClient
    """
    try:
        # 创建客户端实例
        client = DemoServerClient()
        
        # 示例数据 - 你可以根据实际需要修改这些数据
        demo_data = {
            "action": "transfer",
            "platform": "demo_platform",
            "timestamp": "2025-11-11T00:00:00Z"
        }
        
        # 发送请求
        result = client.transfer_server_onto_platform(demo_data)
        
        print("="*50)
        print("请求发送成功!")
        print(f"响应结果: {json.dumps(result, indent=2, ensure_ascii=False)}")
        print("="*50)
        
    except Exception as e:
        print("="*50)
        print(f"请求失败: {str(e)}")
        print("="*50)
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
