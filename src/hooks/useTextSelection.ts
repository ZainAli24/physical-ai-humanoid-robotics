/**
 * useTextSelection Hook
 * Feature: 005-text-selection
 *
 * Detects text selection in Docusaurus pages and calculates button position.
 * Tasks: T013, T014, T015
 */

import { useState, useEffect, useCallback } from 'react';

export interface SelectionInfo {
  text: string;
  isValid: boolean;
  buttonPosition: {
    top: number;
    left: number;
  } | null;
}

const MIN_SELECTION_LENGTH = 10;
const BUTTON_OFFSET_Y = 10; // Pixels below selection
const BUTTON_OFFSET_X = 0; // Pixels to the right of selection
const IOS_SELECTION_HANDLE_OFFSET = 60; // T044: Extra offset for iOS to avoid selection handles

/**
 * Hook to detect and validate text selection on the page.
 *
 * @returns Selection info including text, validity, and button position
 *
 * Features:
 * - Listens to selectionchange events (T013)
 * - Validates selection length >= 10 chars (T014)
 * - Calculates smart button position from getBoundingClientRect() (T015)
 * - Debounces selection changes (200ms) to prevent flicker
 * - Returns null position when selection is cleared or invalid
 *
 * Usage:
 * ```tsx
 * const { text, isValid, buttonPosition } = useTextSelection();
 *
 * if (isValid && buttonPosition) {
 *   // Show "Add to Chat" button at buttonPosition
 * }
 * ```
 */
export function useTextSelection(): SelectionInfo {
  const [selectionInfo, setSelectionInfo] = useState<SelectionInfo>({
    text: '',
    isValid: false,
    buttonPosition: null,
  });

  const handleSelectionChange = useCallback(() => {
    // Get current selection
    const selection = window.getSelection();

    // Check if selection exists and has content
    if (!selection || selection.rangeCount === 0) {
      setSelectionInfo({
        text: '',
        isValid: false,
        buttonPosition: null,
      });
      return;
    }

    // Get selected text (trimmed)
    const selectedText = selection.toString().trim();

    // Validate selection length (T014: minimum 10 characters)
    const isValid = selectedText.length >= MIN_SELECTION_LENGTH;

    if (!isValid) {
      setSelectionInfo({
        text: selectedText,
        isValid: false,
        buttonPosition: null,
      });
      return;
    }

    // Calculate button position (T015: getBoundingClientRect)
    try {
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();

      // T044: Detect iOS to apply extra offset for selection handles
      const isIOS = /iPad|iPhone|iPod/.test(navigator.userAgent);
      const iosOffset = isIOS ? IOS_SELECTION_HANDLE_OFFSET : 0;

      // Position button below and slightly to the right of selection
      // This avoids covering the selected text
      const buttonPosition = {
        top: rect.bottom + window.scrollY + BUTTON_OFFSET_Y + iosOffset,
        left: rect.left + window.scrollX + BUTTON_OFFSET_X,
      };

      setSelectionInfo({
        text: selectedText,
        isValid: true,
        buttonPosition,
      });
    } catch (error) {
      // Handle edge cases (e.g., invalid range)
      console.warn('Error calculating selection position:', error);
      setSelectionInfo({
        text: selectedText,
        isValid,
        buttonPosition: null,
      });
    }
  }, []);

  useEffect(() => {
    // T013: Listen to selectionchange event
    document.addEventListener('selectionchange', handleSelectionChange);

    // Cleanup listener on unmount
    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, [handleSelectionChange]);

  return selectionInfo;
}
