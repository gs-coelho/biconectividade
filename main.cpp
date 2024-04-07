#include <cstdio>
#include <set>
#include <stack>
#include <utility>
#include <vector>

#define DEBUG_CLUSTER_STACK false

using std::pair;
using std::set;
using std::stack;
using std::vector;

/// @brief Representa um grafo não-direcionado. Os vértices são indexados em 1.
class Grafo {
 public:
  Grafo(int n) : n_(n), m_(0), grafo_(n + 1, vector<int>()) {}

  /// @brief Cria uma nova aresta (u, v) no grafo não-direcionado
  /// @param u O vértice u da aresta (u, v)
  /// @param v O vértice v da aresta (u, v)
  /// @return `true` se a aresta foi criada corretamente, `false` caso contrário
  bool CriaAresta(int u, int v) {
    // Verifica se os vértices estão dentro do tamanho do grafo antes de criar a
    // aresta.
    if (u <= this->n_ && v <= this->n_) {
      grafo_[u].push_back(v);
      grafo_[v].push_back(u);
      m_++;
      return true;
    }

    return false;
  }

  /// @brief Encontra os pontos de articulação do grafo não-direcionado
  /// @return Um `vector<bool>` com valores `true` se o vértice for um ponto de
  /// articulação, e `false` caso contrário.
  pair<int, vector<bool>> EncontraArticulacao() {
    timer_ = 0;
    qtd_articulacao_ = 0;

    visitado_.assign(n_ + 1, false);
    articulacao_.assign(n_ + 1, false);

    t_abertura_.assign(n_ + 1, -1);
    low_point_.assign(n_ + 1, -1);

    clusters_.clear();
    cluster_atual_ = stack<pair<int, int>>();

    for (int i = n_; i >= 1; i--) {
      if (!visitado_[i]) {
        dfs_articulacao_(i);
      }
    }

    return {qtd_articulacao_, articulacao_};
  }

  /// @brief Retorna o conjunto de clusters do grafo
  /// @return O conjunto de clusters do grafo
  set<set<int>> GetClusters() { return clusters_; }

 private:
  // `n_` = número de vértices do grafo, `m_` = número de arestas do grafo
  int n_, m_;

  // Lista de adjacências do grafo
  vector<vector<int>> grafo_;

  // Guarda os tempos de abertura de cada vértice na DFS
  vector<int> t_abertura_;

  // Guarda o lowpoint de cada vértice na DFS
  vector<int> low_point_;

  // Vector com valores `true` se um vértice já foi visitado, `false` caso
  // contrário
  vector<bool> visitado_;

  // Vector com valores `true` se um vértice é um ponto de articulação, `false`
  // caso contrário
  vector<bool> articulacao_;

  // Conjunto de clusters do grafo
  set<set<int>> clusters_;

  // Guarda as arestas percorridas, que virão a se tornar clusters quando um
  // ponto de articulação for encontrado.
  stack<pair<int, int>> cluster_atual_;

  // O tempo atual durante a DFS, utilizado para marcar os tempos de abertura e
  // fechamento
  int timer_;

  // Quantidade de pontos de articulação do grafo
  int qtd_articulacao_;

  /// @brief Processa o vértice `v` a partir do vértice `pai` na DFS
  /// @param v O novo vértice a ser processado
  /// @param pai O vértice a partir de qual `v` foi descoberto
  void dfs_articulacao_(int v, int pai = -1) {
    // Abre o vértice e coloca a aresta utilizada na pilha do cluster atual
    timer_++;
    visitado_[v] = true;
    t_abertura_[v] = low_point_[v] = timer_;
    cluster_atual_.push({pai, v});

    if (DEBUG_CLUSTER_STACK) {
      printf("+(%2d,%2d)\n", pai, v);
    }

    int filhos = 0;
    for (int w : grafo_[v]) {
      // Se a aresta voltar diretamente para o pai, pule
      if (w == pai) {
        continue;
      }

      if (visitado_[w]) {  // Caso a aresta seja de retorno
        // Atualiza o low_point do vértice atual
        if (t_abertura_[w] < low_point_[v]) {
          low_point_[v] = t_abertura_[w];
        }
      } else {  // Se for uma aresta de árvore
        // Coloca o vértice atual na pilha do cluster e visita o novo filho
        dfs_articulacao_(w, v);
        filhos++;

        // Se algum descendente tiver encontrado um low_point menor, atualiza o
        // low_point do vértice atual
        if (low_point_[w] < low_point_[v]) {
          low_point_[v] = low_point_[w];
        }

        // Se nenhum descendente puder voltar para algum ascendente do vértice
        // atual, ele é candidato a ponto de articulação
        if ((low_point_[w] >= t_abertura_[v])) {
          // Se o vértice não for a raiz, ele é um ponto de articulação (a raiz
          // também satisfaz a condição acima, mas ela tem uma condição
          // adicional para ser considerada ponto de articulação, que só pode
          // ser verificada ao final da DFS)
          if (pai != -1) {
            if (!articulacao_[v]) {
              articulacao_[v] = true;
              qtd_articulacao_++;
            }
          }

          // Transforma a subárvore recém-visitada em um cluster
          set<int> cluster_temp({v});
          auto t = cluster_atual_.top();
          for (t; t.first != v; t = cluster_atual_.top()) {
            cluster_temp.insert(t.second);
            cluster_atual_.pop();

            if (DEBUG_CLUSTER_STACK) {
              printf("-(%2d,%2d)\n", t.first, t.second);
            }
          }

          cluster_temp.insert(t.second);
          cluster_atual_.pop();
          clusters_.insert(cluster_temp);
        }
      }
    }

    // Caso especial: se a raiz da DFS tiver mais de um filho na árvore, ela é
    // um ponto de articulação
    if (pai == -1 && filhos > 1) {
      if (!articulacao_[v]) {
        articulacao_[v] = true;
        qtd_articulacao_++;
      }
    }
  }
};

int main() {
  int n, m;
  scanf("%d %d", &n, &m);
  Grafo g(n);

  // Cria as arestas a partir da entrada
  int u, v;
  for (int i = 0; i < m; i++) {
    scanf("%d %d", &u, &v);
    g.CriaAresta(u, v);
  }

  // Encontra os pontos de articulação do grafo
  pair<int, vector<bool>> tmp_cutpoints = g.EncontraArticulacao();
  int qtd_cutpoints = tmp_cutpoints.first;
  vector<bool> cutpoints = tmp_cutpoints.second;

  // Imprime os pontos de articulação do grafo
  printf("%d\n", qtd_cutpoints);
  for (int i = 1; i <= n; i++) {
    if (cutpoints[i]) {
      printf("%d\n", i);
    }
  }

  // Obtém os clusters encontrados ao buscar pelos pontos de articulação
  set<set<int>> clusters = g.GetClusters();
  set<pair<int, int>> floresta;

  // Imprime os clusters gerados com seus respectivos índices, e ao mesmo tempo,
  // gera as arestas da floresta de clusters-bordas do grafo.
  int idx = n + 1;
  printf("%ld\n", clusters.size());
  for (auto it = clusters.cbegin(); it != clusters.cend(); it++, idx++) {
    printf("%d %ld", idx, it->size());

    for (int v : *it) {
      printf(" %d", v);

      // Se um cutpoint faz parte de um cluster, deve existir uma aresta entre
      // ele e o cluster de que ele faz parte.
      if (cutpoints[v]) {
        floresta.insert({v, idx});
      }
    }

    printf("\n");
  }

  // Imprime a floresta de clusters-bordas gerada no passo anterior.
  printf("%ld %ld\n", qtd_cutpoints + clusters.size(), floresta.size());
  for (auto it = floresta.cbegin(); it != floresta.cend(); it++) {
    printf("%d %d\n", it->first, it->second);
  }

  return 0;
}
