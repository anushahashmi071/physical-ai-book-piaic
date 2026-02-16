"""
Script to optimize embeddings and retrieval parameters based on content characteristics.
This script analyzes the content and suggests optimal parameters for the RAG system.
"""

import json
from collections import Counter
import math


def analyze_content_characteristics():
    """
    Analyze the characteristics of the processed content to inform parameter optimization.
    """
    print("Analyzing content characteristics...")

    # Load processed content
    try:
        with open("processed_textbook_content.json", 'r', encoding='utf-8') as f:
            content = json.load(f)
        print(f"Loaded {len(content)} content chunks for analysis")
    except FileNotFoundError:
        print("Error: processed_textbook_content.json not found")
        return None

    # Analyze content characteristics
    content_lengths = []
    word_counts = []
    char_counts = []
    titles = []
    chapters = []

    for item in content:
        content_text = item.get('content', '')
        title = item.get('title', '')

        content_lengths.append(len(content_text))
        word_counts.append(len(content_text.split()))
        char_counts.append(len(content_text))
        titles.append(title)
        chapters.append(item.get('chapter', 'unknown'))

    # Calculate statistics
    avg_length = sum(content_lengths) / len(content_lengths) if content_lengths else 0
    median_length = sorted(content_lengths)[len(content_lengths)//2] if content_lengths else 0
    max_length = max(content_lengths) if content_lengths else 0
    min_length = min(content_lengths) if content_lengths else 0

    avg_words = sum(word_counts) / len(word_counts) if word_counts else 0
    avg_chars = sum(char_counts) / len(char_counts) if char_counts else 0

    # Chapter distribution
    chapter_counts = Counter(chapters)

    print(f"\nContent Analysis Results:")
    print(f"- Total content chunks: {len(content_lengths)}")
    print(f"- Average content length: {avg_length:.2f} characters ({avg_words:.2f} words)")
    print(f"- Median content length: {median_length:.2f} characters")
    print(f"- Content length range: {min_length}-{max_length} characters")
    print(f"- Average characters per chunk: {avg_chars:.2f}")
    print(f"- Unique chapters: {len(chapter_counts)}")
    print(f"- Most common chapters: {chapter_counts.most_common(5)}")

    return {
        'content_lengths': content_lengths,
        'word_counts': word_counts,
        'char_counts': char_counts,
        'chapters': chapters,
        'avg_length': avg_length,
        'median_length': median_length,
        'max_length': max_length,
        'min_length': min_length,
        'avg_words': avg_words,
        'avg_chars': avg_chars,
        'chapter_counts': chapter_counts
    }


def suggest_optimal_parameters(content_analysis):
    """
    Suggest optimal parameters based on content characteristics.
    """
    print(f"\nSuggesting optimal parameters based on content analysis...")

    if not content_analysis:
        print("No content analysis data available")
        return

    avg_length = content_analysis['avg_length']
    avg_words = content_analysis['avg_words']
    max_length = content_analysis['max_length']
    chapter_counts = content_analysis['chapter_counts']

    print(f"\nParameter Optimization Suggestions:")

    # Suggest chunk size based on average content length
    suggested_chunk_size = max(300, min(1000, int(avg_length * 1.2)))
    print(f"- Chunk size: {suggested_chunk_size} characters (based on avg length of {avg_length:.0f})")

    # Suggest overlap based on chunk size
    suggested_overlap = max(30, min(200, suggested_chunk_size // 4))
    print(f"- Overlap: {suggested_overlap} characters ({suggested_overlap/suggested_chunk_size*100:.1f}% of chunk)")

    # Suggest embedding model based on content type
    if avg_words < 50:
        embedding_suggestion = "all-MiniLM-L6-v2 (lightweight, good for short text)"
    elif avg_words < 150:
        embedding_suggestion = "all-mpnet-base-v2 (balanced performance)"
    else:
        embedding_suggestion = "sentence-t5-xxl (high quality for longer text)"

    print(f"- Recommended embedding model: {embedding_suggestion}")

    # Suggest retrieval parameters
    total_chunks = len(content_analysis['content_lengths'])
    if total_chunks < 1000:
        top_k = 3
    elif total_chunks < 5000:
        top_k = 5
    else:
        top_k = 7

    print(f"- Top-K retrieval: {top_k} (based on total {total_chunks} chunks)")

    # Suggest similarity threshold based on content diversity
    if len(chapter_counts) > 10:  # Many different chapters, diverse content
        threshold = 0.5
        print(f"- Similarity threshold: {threshold} (diverse content requires lower threshold)")
    else:
        threshold = 0.6
        print(f"- Similarity threshold: {threshold} (focused content allows higher threshold)")

    # Suggest indexing parameters
    print(f"- Indexing strategy: Recommend using HNSW index for fast retrieval")
    print(f"- Quantization: Consider product quantization for large collections to save memory")

    # Performance recommendations
    print(f"\nPerformance Recommendations:")
    print(f"- For queries: Use batch processing for multiple simultaneous requests")
    print(f"- For storage: Monitor vector database memory usage as content grows")
    print(f"- For latency: Consider caching frequent queries")

    return {
        'suggested_chunk_size': suggested_chunk_size,
        'suggested_overlap': suggested_overlap,
        'recommended_model': embedding_suggestion,
        'top_k': top_k,
        'similarity_threshold': threshold
    }


def create_optimization_report(content_analysis, suggestions):
    """
    Create an optimization report with the findings and suggestions.
    """
    print(f"\nCreating optimization report...")

    report = {
        "analysis_date": "2026-01-23",
        "total_chunks_analyzed": len(content_analysis['content_lengths']) if content_analysis else 0,
        "average_content_length": content_analysis['avg_length'] if content_analysis else 0,
        "suggested_parameters": suggestions,
        "recommendations": [
            "Monitor system performance after implementing suggested parameters",
            "Adjust similarity threshold based on user feedback",
            "Consider content growth when planning infrastructure",
            "Regularly reevaluate parameters as content evolves"
        ]
    }

    # Save report to file
    with open("optimization_report.json", 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2)

    print(f"Optimization report saved to optimization_report.json")
    return report


def run_optimization():
    """
    Run the full optimization process.
    """
    print("Starting parameter optimization process...")

    # Analyze content
    content_analysis = analyze_content_characteristics()

    # Suggest optimal parameters
    suggestions = suggest_optimal_parameters(content_analysis)

    # Create optimization report
    report = create_optimization_report(content_analysis, suggestions)

    print(f"\nParameter optimization completed!")
    print(f"Recommendations have been documented in optimization_report.json")
    print(f"This includes content-aware suggestions for chunk size, overlap,")
    print(f"embedding models, and retrieval parameters.")

    return True


def main():
    """
    Main function to run parameter optimization.
    """
    try:
        success = run_optimization()
        return success
    except Exception as e:
        print(f"Error during parameter optimization: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    main()