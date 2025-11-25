#!/bin/bash

echo "📡 Starting Ngrok Tunnel"
echo "===================================="
echo ""
echo "⚠️  IMPORTANTE: Anote a URL que aparecer!"
echo ""

# Verificar se ngrok está instalado
if ! command -v ngrok &> /dev/null; then
    echo "❌ Ngrok não encontrado!"
    echo "Instale com: sudo snap install ngrok"
    exit 1
fi

# Verificar se está autenticado
if ! ngrok config check &> /dev/null; then
    echo "⚠️  Ngrok não autenticado!"
    echo ""
    echo "1. Acesse: https://dashboard.ngrok.com/get-started/your-authtoken"
    echo "2. Copie seu token"
    echo "3. Execute: ngrok config add-authtoken SEU_TOKEN"
    echo ""
    exit 1
fi

# Iniciar túnel
echo "🚀 Iniciando túnel na porta 8012..."
echo ""
ngrok http 8012
