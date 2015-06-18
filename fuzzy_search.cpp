#include <algorithm>
#include <deque>
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <unordered_set>
#include <iterator>
#include <sstream>
#include <memory>
#include <utility>

//  std::make_unique will be available since c++14
//  Implementation was taken from http://herbsutter.com/gotw/_102/
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template<class Iterator>
class IteratorRange {
public:
    IteratorRange(Iterator begin, Iterator end):
        begin_(begin), end_(end) {}

    Iterator begin() const {
        return begin_;
    }
    Iterator end() const {
        return end_;
    }
private:
    Iterator begin_, end_;
};

namespace traverses {

template<class Visitor, class Graph, class Vertex>
void BreadthFirstSearch(Vertex origin_vertex, const Graph& graph, Visitor visitor)
{
    std::queue<Vertex> processed_vertices;
    std::unordered_set<Vertex> visited;
    visitor.DiscoverVertex(origin_vertex);
    visited.insert(origin_vertex);
    processed_vertices.push(origin_vertex);
    while (!processed_vertices.empty()) {
        Vertex current_vertex = processed_vertices.front();
        processed_vertices.pop();
        visitor.ExamineVertex(current_vertex);
        auto edges = graph.OutgoingEdges(current_vertex);
        for (auto&& edge : edges) {
            Vertex next_vertex = GetTarget(edge);
            visitor.ExamineEdge(edge);
            if (!visited.count(next_vertex)) {
               visited.insert(next_vertex); 
               visitor.DiscoverVertex(next_vertex);
               processed_vertices.push(next_vertex);
            }
        }
    }
}

// See "Visitor Event Points" on
// http://www.boost.org/doc/libs/1_57_0/libs/graph/doc/breadth_first_search.html
template<class Vertex, class Edge>
class BfsVisitor {
public:
    virtual void DiscoverVertex(Vertex /*vertex*/) {}
    virtual void ExamineEdge(const Edge& /*edge*/) {}
    virtual void ExamineVertex(Vertex /*vertex*/) {}
    virtual ~BfsVisitor() {}
};

}  // namespace traverses

namespace aho_corasick {

struct AutomatonNode {
    AutomatonNode():
        suffix_link(nullptr),
        terminal_link(nullptr) {
    }

    std::vector<size_t> matched_string_ids;
    // Stores tree structure of nodes
    std::map<char, AutomatonNode> trie_transitions;
    // Stores pointers to the elements of trie_transitions
    std::map<char, AutomatonNode*> automaton_transitions;
    AutomatonNode* suffix_link;
    AutomatonNode* terminal_link;
};

// Returns nullptr if there is no such transition
AutomatonNode* GetTrieTransition(AutomatonNode* node, char character)
{
    auto char_and_node = node->trie_transitions.find(character);
    if (char_and_node == node->trie_transitions.end()) {
        return nullptr;
    } else {
        AutomatonNode* transition = &char_and_node->second;
        return transition;
    }
}

// Performs transition in automaton
AutomatonNode* GetNextNode(AutomatonNode* node, AutomatonNode* root, char character)
{
    if (node->automaton_transitions[character]) {
        return node->automaton_transitions[character];
    }
    AutomatonNode* transition = GetTrieTransition(node, character);
    if (!transition) {
        if (node != root) {
            transition = GetNextNode(node->suffix_link, root, character);
        } else {
            transition = root;
        }
    }
    node->automaton_transitions[character] = transition;
    return transition;
}

namespace internal {

class AutomatonGraph;

class AutomatonGraph {
public:
    struct Edge {
        Edge(AutomatonNode* source,
                AutomatonNode* target,
                char character):
            source(source),
            target(target),
            character(character) {
        }

        AutomatonNode* source;
        AutomatonNode* target;
        char character;
    };

    // Returns edges corresponding to all trie transitions from vertex
    std::vector<Edge> OutgoingEdges(AutomatonNode* vertex) const {
        std::vector <Edge> edges;
        for (auto& pair : vertex->trie_transitions) {
            edges.push_back(Edge(vertex, &pair.second, pair.first));
        }    
        return edges;
    }
};

AutomatonNode* GetTarget(const AutomatonGraph::Edge& edge)
{
    return edge.target;
}

class SuffixLinkCalculator:
    public traverses::BfsVisitor<AutomatonNode*, AutomatonGraph::Edge> {
public:
    explicit SuffixLinkCalculator(AutomatonNode* root):
        root_(root) {}

    void ExamineVertex(AutomatonNode* node) /*override*/ {
        if (node == root_) {
            node->suffix_link = root_;
        }
    }

    void ExamineEdge(const AutomatonGraph::Edge& edge) /*override*/ {
        if (edge.source == root_) {
            edge.target->suffix_link = root_;
            return;
        }
        edge.target->suffix_link = GetNextNode(edge.source->suffix_link, root_, edge.character);
    }

private:
    AutomatonNode* root_;
};

class TerminalLinkCalculator:
    public traverses::BfsVisitor<AutomatonNode*, AutomatonGraph::Edge> {
public:
    explicit TerminalLinkCalculator(AutomatonNode* root):
        root_(root) {}

    void DiscoverVertex(AutomatonNode* node) /*override*/ {
        if (node != root_) {
            if ((node->suffix_link)->matched_string_ids.empty()) {
               node->terminal_link = node->suffix_link->terminal_link; 
            } else {
               node->terminal_link = node->suffix_link; 
            }
        } else {
            node->terminal_link = nullptr;
        }
    }

private:
    AutomatonNode* root_;
};

}  // namespace internal

class NodeReference {
public:
    typedef std::vector<size_t>::const_iterator MatchedStringIterator;
    typedef IteratorRange<MatchedStringIterator> MatchedStringIteratorRange;

    NodeReference():
        node_(nullptr),
        root_(nullptr) {
    }
    NodeReference(AutomatonNode* node, AutomatonNode* root):
        node_(node), root_(root) {
    }

    NodeReference Next(char character) const {
        return NodeReference(GetNextNode(node_, root_, character), root_);
    }

    NodeReference suffixLink() const {
        return NodeReference(node_->suffix_link, root_);
    }

    NodeReference terminalLink() const {
        return NodeReference(node_->terminal_link, root_);
    }

    MatchedStringIteratorRange matchedStringIds() const {
        return MatchedStringIteratorRange(node_->matched_string_ids.begin(), 
                                          node_->matched_string_ids.end());
    }

    explicit operator bool() const {
        return node_ != nullptr;
    }

    bool operator==(NodeReference other) const {
        return (node_ == other.node_ && root_ == other.root_);
    }

private:
    AutomatonNode* node_;
    AutomatonNode* root_;
};

using std::rel_ops::operator !=;

class AutomatonBuilder;

class Automaton {
public:
    Automaton() = default;

    NodeReference Root() {
        return NodeReference(&root_, &root_);
    }

    template <class Callback>
    void GenerateMatches(NodeReference node, Callback on_match) {
        while (node) {
            for (auto id : node.matchedStringIds()) {
                on_match(id);
            }
            node = node.terminalLink();
        }
    }

private:
    AutomatonNode root_;
    Automaton(const Automaton&) = delete;
    Automaton& operator=(const Automaton&) = delete;
    friend class AutomatonBuilder;
};

class AutomatonBuilder {
public:
    void Add(const std::string& string, size_t id) {
        words_.push_back(string);
        ids_.push_back(id);
    }

    std::unique_ptr<Automaton> Build() {
        auto automaton = make_unique<Automaton>();
        BuildTrie(words_, ids_, automaton.get());
        BuildSuffixLinks(automaton.get());
        BuildTerminalLinks(automaton.get());
        return automaton;
    }

private:
    static void BuildTrie(const std::vector<std::string>& words,
            const std::vector<size_t>& ids,
            Automaton* automaton) {
        for (size_t i = 0; i < words.size(); ++i) {
            AddString(&automaton->root_, ids[i], words[i]);
        }
    }

    static void AddString(AutomatonNode* root, size_t string_id, const std::string& string) {
        AutomatonNode* node = root;
        for (char c : string) {
            node = &node->trie_transitions[c];
        }
        node->matched_string_ids.push_back(string_id);
    }

    static void BuildSuffixLinks(Automaton* automaton) {
        internal::SuffixLinkCalculator visitor(&automaton->root_);
        traverses::BreadthFirstSearch(&automaton->root_, internal::AutomatonGraph(), visitor);
    }

    static void BuildTerminalLinks(Automaton* automaton) {
        internal::TerminalLinkCalculator visitor(&automaton->root_);
        traverses::BreadthFirstSearch(&automaton->root_, internal::AutomatonGraph(), visitor);
    }

    std::vector<std::string> words_;
    std::vector<size_t> ids_;
};

}  // namespace aho_corasick

// Consecutive delimiters are not grouped together and are deemed
// to delimit empty strings
template <class Predicate>
std::vector<std::string> Split(const std::string& string, Predicate is_delimiter) {
    std::vector<std::string> split_string;
    auto left = string.begin();
    auto right = left;
    while (right != string.end()) {
        right = std::find_if(left, string.end(), is_delimiter);
        split_string.emplace_back(left, right);
        if (right != string.end()) {
            left = std::next(right);
        }
    }
    return split_string;
}

// Wildcard is a character that may be substituted
// for any of all possible characters
class WildcardMatcher {
public:
    WildcardMatcher():
        number_of_words_(0),
        pattern_length_(0) {
    }

    void Init(const std::string& pattern, char wildcard) {
        std::vector<std::string> split_pattern = Split(pattern, [wildcard](char c) { 
            return c == wildcard;
        });
        pattern_length_ = pattern.size();
        number_of_words_ = split_pattern.size();
        aho_corasick::AutomatonBuilder builder;

        int offset = 0;
        for (auto&& word : split_pattern) {
            offset += word.size();
            builder.Add(word, offset);
            offset++; // one symbol for wildcard
        }
        aho_corasick_automaton_ = builder.Build();
        Reset();
    }

    // Saves found matches in position, relative to length of the pattern.
    void AddMatches() {
        auto& occurrences_link = words_occurrences_by_position_;
        int pattern_length = pattern_length_;
        aho_corasick_automaton_->GenerateMatches(state_, [&occurrences_link, 
                                                          pattern_length](int pos) {
            occurrences_link[pattern_length - pos] += 1;
        });
    }

    // Resets matcher to start scanning new stream
    void Reset() {
        state_ = aho_corasick_automaton_->Root();
        words_occurrences_by_position_.assign(pattern_length_ + 1, 0);
        // Initializing matches in root.
        AddMatches();
    }

    template<class Callback>
    void Scan(char character, Callback on_match) {
        state_ = state_.Next(character);
        words_occurrences_by_position_.push_back(0);
        words_occurrences_by_position_.pop_front();
        AddMatches();
        if (words_occurrences_by_position_[0] == number_of_words_) {
            on_match();
        }        
    }

private:
    // Storing only O(|pattern|) elements allows us
    // to consume only O(|pattern|) memory for matcher
    std::deque<size_t> words_occurrences_by_position_;
    aho_corasick::NodeReference state_;
    size_t number_of_words_;
    size_t pattern_length_;
    std::unique_ptr<aho_corasick::Automaton> aho_corasick_automaton_;
};

std::string ReadString(std::istream& input_stream) {
    std::string str;
    input_stream >> str;
    return str;
}

// Returns positions of the first character of every match
std::vector<size_t> FindFuzzyMatches(const std::string& patternWithWildcards,
        const std::string& text,
        char wildcard) {
    WildcardMatcher matcher;
    matcher.Init(patternWithWildcards, wildcard);
    std::vector<size_t> occurrences;
    for (size_t offset = 0; offset < text.size(); ++offset) {
        matcher.Scan(text[offset],
            [&occurrences, offset, &patternWithWildcards] {
                occurrences.push_back(offset + 1 - patternWithWildcards.size());
            });
    }
    return occurrences;
}

void Print(const std::vector<size_t>& sequence) {
    std::cout << sequence.size() << "\n";
    for (int i = 0; i < sequence.size(); ++i) {
        std::cout << sequence[i] << " ";
    }
    std::cout << "\n";
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    // freopen("input.txt", "r", stdin);
    const char wildcard = '?';
    const std::string patternWithWildcards = ReadString(std::cin);
    const std::string text = ReadString(std::cin);
    Print(FindFuzzyMatches(patternWithWildcards, text, wildcard));
    return 0;
}
